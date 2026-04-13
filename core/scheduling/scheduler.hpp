/**
 * @file scheduler.hpp
 * @brief Cooperative task scheduler — priority-ordered immediate + timer-deferred tasks.
 *
 * All task queues use ETL static containers (no heap allocation).
 * Thread-safe via InterruptLock on all queue operations.
 */

#pragma once

#include <optional>
#include <memory>

#include "interrupt_lock.hpp"
#include "timer.hpp"
#include "debug.hpp"
#include "inplace_function.hpp"
#include "etl/list.h"
#include "etl/vector.h"

static constexpr unsigned int MAX_NUM_TASKS = 64;  ///< Max concurrent pending tasks

#ifndef INPLACE_FUNCTION_SIZE_SCHEDULER
#define INPLACE_FUNCTION_SIZE_SCHEDULER 12
#endif

/// @brief Cooperative task scheduler with priority ordering and timer-deferred execution.
class Scheduler {

public:
	static constexpr unsigned int HIGHEST_PRIORITY = 0;
	static constexpr unsigned int DEFAULT_PRIORITY = 7;

	Scheduler(Timer *timer) : m_timer(timer), m_unique_id(0) {}
	Scheduler(const Scheduler &) = delete;
	
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"

	class Task
    {
		friend class Scheduler;

	private:
		const char *m_name;
		std::optional<unsigned int> m_id;
		unsigned int m_priority;
		stdext::inplace_function<void(), INPLACE_FUNCTION_SIZE_SCHEDULER> m_func;
    };
#pragma GCC diagnostic pop

	/// @brief Opaque handle to a posted task — used for cancellation and status queries.
	class TaskHandle
	{
		friend class Scheduler;

	public:
		TaskHandle() = default;

	private:
		const char *m_name = nullptr;
		Scheduler *m_parent = nullptr;
		std::optional<unsigned int> m_id;
	};

	/// @brief Post a task for execution — immediate or after a delay.
	/// @param task_func  Callback to execute (stored inline, no heap).
	/// @param task_name  Debug name (must be a string literal — not copied).
	/// @param priority   Lower = higher priority (0 = highest, 7 = default).
	/// @param delay_ms   0 = immediate, >0 = deferred via hardware timer.
	/// @return Handle for cancellation / status check.
	TaskHandle post_task_prio(stdext::inplace_function<void(), INPLACE_FUNCTION_SIZE_SCHEDULER> const &task_func, const char *task_name, unsigned int priority = DEFAULT_PRIORITY, unsigned int delay_ms = 0) {

		Task task;
		task.m_priority = priority;
		task.m_func = task_func;
		task.m_name = task_name;

		// Safely create a new unique ID for this task
		{
			InterruptLock lock;
			task.m_id = m_unique_id++; // Incrementing this is non-atomic so we must do so within a lock

			if (!delay_ms)
				schedule_now(task);
			else
				schedule_deferred(task, delay_ms);
		}

		TaskHandle handle;
		handle.m_id = task.m_id;
		handle.m_parent = this;
		handle.m_name = task_name;

#ifdef SCHEDULER_DEBUG
		DEBUG_TRACE("Scheduler: post_task_prio: added %s", task_name);
#endif
		return handle;
	}

	/// @brief Cancel a pending task (immediate or deferred). Invalidates the handle.
	/// @param task  Handle returned by post_task_prio (safe to call with default handle).
	void cancel_task(TaskHandle &task) {

		if (task.m_parent != this) {
			return; // This handle belongs to another scheduler
		}

		if (!task.m_id.has_value()) {
			return;
		}

		InterruptLock lock;

		// Check to see if this task is in our immediate task list
		auto iter_task = m_tasks.begin();
		while (iter_task != m_tasks.end())
		{
			if (iter_task->m_id == task.m_id)
			{
#ifdef SCHEDULER_DEBUG
				DEBUG_TRACE("Scheduler: cancel_task: %s [immediate]", task.m_name);
#endif
				iter_task = m_tasks.erase(iter_task);
				break;
			}
			else
				iter_task++;
		}

		// Check if this task is in our deferred task list
		// If so then cancel the timer scheduler for it
		auto iter_timer = m_timer_schedules.begin();
		while (iter_timer != m_timer_schedules.end())
		{
			if (iter_timer->id == *task.m_id)
			{
#ifdef SCHEDULER_DEBUG
			DEBUG_TRACE("Scheduler: cancel_task: %s [timer]", task.m_name);
#endif
				m_timer->cancel_schedule(iter_timer->handle);
				iter_timer = m_timer_schedules.erase(iter_timer);
			}
			else
				iter_timer++;
		}

		// Invalidate the task handle in all cases
		task.m_id.reset();
		task.m_parent = nullptr;
	}

	/// @brief Execute all pending immediate tasks in priority order.
	/// @return true if at least one task was executed.
	bool run() {

		// Run through our queue of tasks in order and run them
		// As our queue is in priority order so will our run order
		bool tasks_ran = false;

		while (true)
		{
			// We need to safetly retrieve the front value with a lock
			// We then need to release the lock so that the called function may add/cancel tasks

			Task task;
			{
				InterruptLock lock;
				if (!m_tasks.size())
					return tasks_ran;

				task = m_tasks.front();
				m_tasks.pop_front();
			}

			if (task.m_func) {
#ifdef SCHEDULER_DEBUG
				DEBUG_TRACE("Scheduler: run: %s", task.m_name);
#endif
				tasks_ran = true;
				task.m_func();
			}
		}
	}

	/// @brief Check if a task is still pending (immediate or deferred).
	/// @param task  Handle to check.
	/// @return true if the task is in either queue.
	bool is_scheduled(TaskHandle task) {
		if (task.m_parent != this)
			return false; // This handle belongs to another scheduler

		if (!task.m_id.has_value())
			return false;
		
		InterruptLock lock;

		// Check to see if this task is in our immediate task list
		auto iter = m_tasks.begin();
		while (iter != m_tasks.end())
		{
			if (iter->m_id == task.m_id)
			{
				return true;
			}
			else
				iter++;
		}

		// Check if this task is in our deferred task list
		for (auto const& value: m_timer_schedules)
			if (value.id == *task.m_id)
				return true;

		return false;
	}

	/// @brief Cancel all pending tasks (immediate + deferred). Used during shutdown.
	void clear_all()
	{
		InterruptLock lock;
		m_tasks.clear();

		// Cancel all scheduled tasks
		auto iter = m_timer_schedules.begin();
		while (iter != m_timer_schedules.end())
		{
			m_timer->cancel_schedule(iter->handle);
			iter = m_timer_schedules.erase(iter);
		}
	}
	
	/// @brief Check if any task is pending (used by PMU to decide idle depth).
	/// @return true if at least one task exists in either queue.
	bool is_any_task_scheduled()
	{
		InterruptLock lock;
		return m_tasks.size() + m_timer_schedules.size();
	}

	/// @brief Time in ms until the earliest deferred task fires (UINT64_MAX if none).
	/// @return Milliseconds until next task, or 0 if immediate tasks pending.
	uint64_t ms_until_next_task()
	{
		InterruptLock lock;
		if (m_tasks.size()) return 0;
		if (m_timer_schedules.empty()) return UINT64_MAX;

		uint64_t now = m_timer->get_counter();
		uint64_t earliest = UINT64_MAX;
		for (auto const& entry : m_timer_schedules) {
			if (entry.target <= now) return 0;
			uint64_t delta = entry.target - now;
			if (delta < earliest) earliest = delta;
		}
		return earliest;
	}

private:

	struct DeferredEntry {
		unsigned int id;
		Timer::TimerHandle handle;
		uint64_t target;
	};

	void schedule_deferred(Task task, unsigned int delay_ms) {
		InterruptLock lock;
		if (m_timer_schedules.full()) {
			DEBUG_ERROR("Scheduler: deferred queue FULL (%u/%u) — dropping '%s'",
			            (unsigned)m_timer_schedules.size(), (unsigned)m_timer_schedules.max_size(), task.m_name);
			return;
		}
		// If this task was delayed then schedule a timer to start it
		uint64_t t_sched = m_timer->get_counter() + delay_ms;

		// We do this by setting up our timer to call this function after the delay has elapsed
		unsigned int id = *task.m_id;
		Timer::TimerHandle handle = m_timer->add_schedule([this, id, task]() {
			this->timer_callback_handler(id, task);
		}, t_sched);

		m_timer_schedules.push_back({id, handle, t_sched});
	}

	void schedule_now(Task task) {
		InterruptLock lock;
		if (m_tasks.full()) {
			DEBUG_ERROR("Scheduler: task queue FULL (%u/%u) — dropping '%s'",
			            (unsigned)m_tasks.size(), (unsigned)m_tasks.max_size(), task.m_name);
			return;
		}
		// Task is requested to be processed on next run()
		// Safetly add this task to our task list in priority order
		auto iter = m_tasks.begin();
		while (iter != m_tasks.end())
		{
			if (iter->m_priority > task.m_priority)
				break;
			iter++;
		}
		m_tasks.insert(iter, task);
	}

	void timer_callback_handler(unsigned int task_id, Task task) {
		auto iter = m_timer_schedules.begin();
		while (iter != m_timer_schedules.end())
		{
			if (iter->id == task_id)
			{
				iter = m_timer_schedules.erase(iter);
			}
			else
				iter++;
		}
		
		schedule_now(task);
	}

	etl::list<Task, MAX_NUM_TASKS> m_tasks;
	etl::vector<DeferredEntry, MAX_NUM_TASKS> m_timer_schedules;
	Timer *m_timer;
	unsigned int m_unique_id;
};
