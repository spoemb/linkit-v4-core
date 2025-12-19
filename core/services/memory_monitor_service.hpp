#include "service.hpp"
#include "memmang.hpp"
#include "nrf_i2c.hpp"

class MemoryMonitorService : public Service {
public:
	MemoryMonitorService() : Service(ServiceIdentifier::MEMORY_MONITOR, "MEMORY") {
	}

	void service_initiate() {
		// Report heap statistics
		HeapStats_t heap_stats = MEMMANG::heap_stats();
		DEBUG_INFO("MemoryMonitorService: HEAP: %d min, %d free, %d freeblk, %d allocs, %d frees",
			heap_stats.xMinimumEverFreeBytesRemaining,
			heap_stats.xAvailableHeapSpaceInBytes,
			heap_stats.xNumberOfFreeBlocks,
			heap_stats.xNumberOfSuccessfulAllocations,
			heap_stats.xNumberOfSuccessfulFrees);
		DEBUG_INFO("MemoryMonitorService: STACK: %u max", MEMMANG::max_stack_usage());

		// Report I2C bus health statistics
		for (uint8_t bus = 0; bus < NrfI2C::num_buses(); bus++) {
			const I2CStats& stats = NrfI2C::get_stats(bus);
			if (stats.total_operations > 0) {
				DEBUG_INFO("I2C[%u]: ops=%u, timeouts=%u, errors=%u, recoveries=%u",
					bus,
					(unsigned int)stats.total_operations,
					(unsigned int)stats.timeouts,
					(unsigned int)stats.errors,
					(unsigned int)stats.bus_recoveries);

				// Warn if error rate exceeds 5%
				if (stats.total_operations > 100) {
					uint32_t error_rate = ((stats.timeouts + stats.errors) * 100) / stats.total_operations;
					if (error_rate > 5) {
						DEBUG_WARN("I2C[%u]: HIGH ERROR RATE: %u%%", bus, error_rate);
					}
				}
			}
		}

		service_complete();
	}

	unsigned int service_next_schedule_in_ms() override {
		// Run every 12 hours
		return 12 * 3600 * 1000;
	}

	bool service_is_enabled() override { return true; }
	bool service_is_usable_underwater() override { return true; }
	void service_init() override {}
	void service_term() override {}
};
