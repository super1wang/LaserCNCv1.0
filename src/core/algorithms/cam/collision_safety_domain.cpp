#include "core/algorithms/cam/collision_safety_domain.h"

#include <QHash>
#include <QQueue>
#include <QReadLocker>
#include <QWriteLocker>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <utility>

namespace lcnc::cam_algo {
namespace {

std::uint64_t doubleBits(double value)
{
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

void hashBytes(std::uint64_t* hash, const void* data, std::size_t size)
{
    constexpr std::uint64_t kPrime = 1099511628211ull;
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
        *hash ^= bytes[index];
        *hash *= kPrime;
    }
}

void hashString(std::uint64_t* hash, const QString& value)
{
    const QByteArray utf8 = value.toUtf8();
    hashBytes(hash, utf8.constData(), static_cast<std::size_t>(utf8.size()));
    const unsigned char terminator = 0xff;
    hashBytes(hash, &terminator, 1);
}

std::uint64_t fingerprint(const CollisionSafetyDomainQuery& query)
{
    std::uint64_t hash = 1469598103934665603ull;
    for (double value : query.pose.kinematicAxes) {
        const std::uint64_t bits = doubleBits(value);
        hashBytes(&hash, &bits, sizeof(bits));
    }
    const std::array<double, 7> geometry{
        query.pose.tcpX, query.pose.tcpY, query.pose.tcpZ,
        query.pose.surfaceNormalX, query.pose.surfaceNormalY,
        query.pose.surfaceNormalZ, query.clearanceMm};
    for (double value : geometry) {
        const std::uint64_t bits = doubleBits(value);
        hashBytes(&hash, &bits, sizeof(bits));
    }
    hashBytes(&hash, &query.pose.kinematicAxisMask,
              sizeof(query.pose.kinematicAxisMask));
    const auto phase = static_cast<std::uint8_t>(query.phase);
    hashBytes(&hash, &phase, sizeof(phase));
    hashString(&hash, query.workpieceEntry);
    hashString(&hash, query.activeSource);
    hashString(&hash, query.passiveSource);
    return hash;
}

bool sameDouble(double lhs, double rhs)
{
    return doubleBits(lhs) == doubleBits(rhs);
}

bool samePose(const lcnc::cam::RapidPose& lhs,
              const lcnc::cam::RapidPose& rhs)
{
    if (lhs.kinematicAxisMask != rhs.kinematicAxisMask) {
        return false;
    }
    for (int index = 0; index < lcnc::MachineAxisLayout::kMaxAxes; ++index) {
        if (!sameDouble(lhs.kinematicAxes[index], rhs.kinematicAxes[index])) {
            return false;
        }
    }
    return sameDouble(lhs.tcpX, rhs.tcpX)
        && sameDouble(lhs.tcpY, rhs.tcpY)
        && sameDouble(lhs.tcpZ, rhs.tcpZ)
        && sameDouble(lhs.surfaceNormalX, rhs.surfaceNormalX)
        && sameDouble(lhs.surfaceNormalY, rhs.surfaceNormalY)
        && sameDouble(lhs.surfaceNormalZ, rhs.surfaceNormalZ);
}

bool sameQuery(const CollisionSafetyDomainQuery& lhs,
               const CollisionSafetyDomainQuery& rhs)
{
    return samePose(lhs.pose, rhs.pose)
        && lhs.workpieceEntry == rhs.workpieceEntry
        && lhs.phase == rhs.phase
        && lhs.activeSource == rhs.activeSource
        && lhs.passiveSource == rhs.passiveSource
        && sameDouble(lhs.clearanceMm, rhs.clearanceMm);
}

} // namespace

struct CollisionSafetyDomainCache::Impl
{
    struct Entry {
        CollisionSafetyDomainQuery query;
        CollisionSafetyDomainSample sample;
    };

    explicit Impl(std::size_t requestedMaximum)
        : maximumSamples(std::max<std::size_t>(1, requestedMaximum))
    {
    }

    mutable QReadWriteLock lock;
    QHash<quint64, QVector<Entry>> entries;
    QQueue<quint64> insertionOrder;
    std::size_t sampleCount{0};
    std::size_t maximumSamples{0};
    mutable std::atomic_uint64_t hits{0};
    mutable std::atomic_uint64_t misses{0};
    std::atomic_uint64_t stores{0};
};

CollisionSafetyDomainCache::CollisionSafetyDomainCache(std::size_t maximumSamples)
    : m_impl(std::make_unique<Impl>(maximumSamples))
{
}

CollisionSafetyDomainCache::~CollisionSafetyDomainCache() = default;

bool CollisionSafetyDomainCache::lookup(
    const CollisionSafetyDomainQuery& query,
    CollisionSafetyDomainSample* sample) const
{
    if (!sample) {
        m_impl->misses.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    const quint64 key = fingerprint(query);
    QReadLocker lock(&m_impl->lock);
    const auto bucket = m_impl->entries.constFind(key);
    if (bucket != m_impl->entries.cend()) {
        for (const Impl::Entry& entry : bucket.value()) {
            if (sameQuery(entry.query, query)) {
                *sample = entry.sample;
                m_impl->hits.fetch_add(1, std::memory_order_relaxed);
                return true;
            }
        }
    }
    m_impl->misses.fetch_add(1, std::memory_order_relaxed);
    return false;
}

void CollisionSafetyDomainCache::store(
    const CollisionSafetyDomainQuery& query,
    const CollisionSafetyDomainSample& sample)
{
    if (!sample.isReusable())
        return;
    const quint64 key = fingerprint(query);
    QWriteLocker lock(&m_impl->lock);
    QVector<Impl::Entry>& bucket = m_impl->entries[key];
    for (Impl::Entry& entry : bucket) {
        if (sameQuery(entry.query, query)) {
            entry.sample = sample;
            return;
        }
    }
    bucket.append({query, sample});
    m_impl->insertionOrder.enqueue(key);
    ++m_impl->sampleCount;
    m_impl->stores.fetch_add(1, std::memory_order_relaxed);
    while (m_impl->sampleCount > m_impl->maximumSamples
           && !m_impl->insertionOrder.isEmpty()) {
        const quint64 oldest = m_impl->insertionOrder.dequeue();
        const auto it = m_impl->entries.find(oldest);
        if (it == m_impl->entries.end())
            continue;
        m_impl->sampleCount -= static_cast<std::size_t>(it.value().size());
        m_impl->entries.erase(it);
    }
}

void CollisionSafetyDomainCache::clear()
{
    QWriteLocker lock(&m_impl->lock);
    m_impl->entries.clear();
    m_impl->insertionOrder.clear();
    m_impl->sampleCount = 0;
}

CollisionSafetyDomainStatistics CollisionSafetyDomainCache::statistics() const
{
    CollisionSafetyDomainStatistics result;
    result.hits = m_impl->hits.load(std::memory_order_relaxed);
    result.misses = m_impl->misses.load(std::memory_order_relaxed);
    result.stores = m_impl->stores.load(std::memory_order_relaxed);
    QReadLocker lock(&m_impl->lock);
    result.samples = m_impl->sampleCount;
    return result;
}

} // namespace lcnc::cam_algo
