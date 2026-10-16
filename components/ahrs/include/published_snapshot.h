#pragma once

#include <stdint.h>

/*
 * A small value handed from one writing task to any number of readers,
 * without a lock.
 *
 * A sequence counter, odd while a write is in progress: the reader
 * retries if it started or finished mid-write, so it can never see half
 * of one value and half of the next. The writer never blocks, which
 * matters when the writer is a sensor loop on a deadline and the reader
 * is a user interface on another core.
 *
 * One writer only. Readers may be many.
 */
template <typename T>
class PublishedSnapshot
{
public:
    PublishedSnapshot() : m_sequence(0), m_value() {}

    void publish(const T &value)
    {
        m_sequence = m_sequence + 1;            /* odd: writing */
        __atomic_thread_fence(__ATOMIC_SEQ_CST);

        m_value = value;

        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        m_sequence = m_sequence + 1;            /* even: consistent */
    }

    T read() const
    {
        T out;
        uint32_t before;
        uint32_t after;

        do
        {
            before = m_sequence;
            __atomic_thread_fence(__ATOMIC_SEQ_CST);

            out = m_value;

            __atomic_thread_fence(__ATOMIC_SEQ_CST);
            after = m_sequence;
        }
        while((before & 1u) != 0u || before != after);

        return out;
    }

private:
    volatile uint32_t m_sequence;
    T m_value;
};
