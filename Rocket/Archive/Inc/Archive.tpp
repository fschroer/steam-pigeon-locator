#pragma once

template<typename TValue>
bool Archive::WriteEvent(FlightArchive::Statistic stat_id, const TValue& value) {
	return archive_.WriteStat(record_id_, stat_id, value);
}

template<typename TValue>
__attribute__((noinline))
bool Archive::ReadEvent(uint16_t record_id, FlightArchive::Statistic statId, TValue& valueOut, bool& presentOut) const {
    return archive_.ReadStat(record_id, static_cast<RocketArchive::StatId>(statId), valueOut, presentOut);
}
