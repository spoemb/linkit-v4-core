#pragma once

#include <vector>
#include <algorithm>

template <typename T>
class EventEmitter {
public:
    void subscribe(T& m) {
        if (std::find(m_listeners.begin(), m_listeners.end(), &m) == m_listeners.end())
            m_listeners.push_back(&m);
    }
    void unsubscribe(T& m) {
        m_listeners.erase(std::remove(m_listeners.begin(), m_listeners.end(), &m), m_listeners.end());
    }

protected:
    template<typename E> void notify(E const& e) {
        auto listeners_copy = m_listeners;
        for (const auto& m : listeners_copy) {
            m->react(e);
        }
    }
    template<typename E> void notify(E & e) {
        auto listeners_copy = m_listeners;
        for (const auto& m : listeners_copy) {
            m->react(e);
        }
    }
private:
    std::vector<T*> m_listeners;

};
