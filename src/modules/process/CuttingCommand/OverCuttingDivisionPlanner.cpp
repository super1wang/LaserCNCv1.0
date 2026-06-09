#include "OverCuttingDivisionPlanner.h"
#include "rs_entity.h"
#include "rs_entitycontainer.h"
#include "rs_vector.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
	const double EPS = 1e-6;

	struct Range
	{
		double left{0.0};
		double right{0.0};
	};

	bool lessRange(const Range& lhs, const Range& rhs)
	{
		if (std::fabs(lhs.left - rhs.left) > EPS)
			return lhs.left < rhs.left;
		return lhs.right < rhs.right;
	}

	std::vector<Range> buildOccupiedRanges(const QList<RS_Entity*>& entityList)
	{
		std::vector<Range> ranges;
		ranges.reserve(static_cast<size_t>(entityList.size()));
		for (RS_Entity* entity : entityList)
		{
			if (!entity || entity->isUndone())
				continue;

			const RS_Vector minPoint = entity->getMin();
			const RS_Vector maxPoint = entity->getMax();
			ranges.push_back({ minPoint.x, maxPoint.x });
		}

		std::stable_sort(ranges.begin(), ranges.end(), lessRange);
		return ranges;
	}

	std::vector<Range> mergeRanges(const std::vector<Range>& ranges)
	{
		std::vector<Range> merged;
		for (const Range& range : ranges)
		{
			if (merged.empty() || range.left > merged.back().right + EPS)
			{
				merged.push_back(range);
			}
			else if (range.right > merged.back().right)
			{
				merged.back().right = range.right;
			}
		}
		return merged;
	}

	bool samePosition(double lhs, double rhs)
	{
		return std::fabs(lhs - rhs) <= EPS;
	}

	void addEndpointCandidate(std::vector<double>& candidates,
		double position,
		double startX,
		double endX)
	{
		if (position <= startX + EPS || position >= endX - EPS)
			return;

		candidates.push_back(position);
	}

	void collectEndpointCandidates(RS_Entity* entity,
		std::vector<double>& candidates,
		double startX,
		double endX)
	{
		if (!entity || entity->isUndone())
			return;

		const RS_Vector startPoint = entity->getStartpoint();
		if (startPoint.valid)
			addEndpointCandidate(candidates, startPoint.x, startX, endX);

		const RS_Vector endPoint = entity->getEndpoint();
		if (endPoint.valid)
			addEndpointCandidate(candidates, endPoint.x, startX, endX);

		if (!entity->isContainer())
			return;

		RS_EntityContainer* container = static_cast<RS_EntityContainer*>(entity);
		for (RS_Entity* child : *container)
			collectEndpointCandidates(child, candidates, startX, endX);
	}

	std::vector<double> buildEndpointCandidates(const QList<RS_Entity*>& entityList,
		double startX,
		double endX)
	{
		std::vector<double> candidates;
		candidates.reserve(static_cast<size_t>(entityList.size()) * 2);
		for (RS_Entity* entity : entityList)
			collectEndpointCandidates(entity, candidates, startX, endX);

		std::stable_sort(candidates.begin(), candidates.end());
		candidates.erase(std::unique(candidates.begin(), candidates.end(), samePosition), candidates.end());
		return candidates;
	}

	bool findBestCut(const std::vector<Range>& occupied,
		double minCut,
		double maxCut,
		double target,
		double& cut)
	{
		bool found = false;
		double bestDistance = std::numeric_limits<double>::max();
		double bestCut = 0.0;
		double cursor = minCut;

		auto tryGap = [&](double gapLeft, double gapRight)
		{
			const double left = std::max(gapLeft, minCut);
			const double right = std::min(gapRight, maxCut);
			if (right - left <= EPS)
				return;

			const double candidate = (left + right) * 0.5;
			const double distance = std::fabs(candidate - target);
			if (!found || distance < bestDistance - EPS ||
				(std::fabs(distance - bestDistance) <= EPS && candidate < bestCut))
			{
				found = true;
				bestDistance = distance;
				bestCut = candidate;
			}
		};

		for (const Range& range : occupied)
		{
			if (range.right <= minCut + EPS)
				continue;
			if (range.left >= maxCut - EPS)
				break;

			tryGap(cursor, range.left);
			if (range.right > cursor)
				cursor = range.right;
		}

		tryGap(cursor, maxCut);
		if (found)
			cut = bestCut;
		return found;
	}

	bool findBestEndpointCut(const std::vector<double>& candidates,
		double minCut,
		double maxCut,
		double target,
		double& cut)
	{
		bool found = false;
		double bestDistance = std::numeric_limits<double>::max();
		double bestCut = 0.0;

		for (double candidate : candidates)
		{
			if (candidate <= minCut + EPS)
				continue;
			if (candidate > maxCut + EPS)
				break;

			const double distance = std::fabs(candidate - target);
			if (!found || distance < bestDistance - EPS ||
				(std::fabs(distance - bestDistance) <= EPS && candidate < bestCut))
			{
				found = true;
				bestDistance = distance;
				bestCut = candidate;
			}
		}

		if (found)
			cut = bestCut;
		return found;
	}

	bool findDivisionCut(const std::vector<Range>& occupied,
		const std::vector<double>& endpointCandidates,
		double minCut,
		double maxCut,
		double target,
		double& cut)
	{
		if (findBestCut(occupied, minCut, maxCut, target, cut))
			return true;

		return findBestEndpointCut(endpointCandidates, minCut, maxCut, target, cut);
	}

	void normalizeDivisions(std::vector<double>& divisions,
		double startX,
		double endX)
	{
		std::stable_sort(divisions.begin(), divisions.end());
		divisions.erase(std::remove_if(divisions.begin(), divisions.end(),
			[&](double point)
			{
				return point <= startX + EPS || point >= endX - EPS;
			}), divisions.end());
		divisions.erase(std::unique(divisions.begin(), divisions.end(), samePosition), divisions.end());
	}

	ErrorCode validateDivisions(const std::vector<double>& divisions,
		double startX,
		double endX,
		double capacity)
	{
		double lastPoint = startX;
		for (double point : divisions)
		{
			const double length = point - lastPoint;
			if (length <= EPS)
				return ErrorCode::ERROR_OVERCUTTING_SEGTOOSHORT;
			if (length > capacity + EPS)
				return ErrorCode::ERROR_OVERCUTTING_NOGAPINLIMIT;
			lastPoint = point;
		}

		if (endX - lastPoint > capacity + EPS)
			return ErrorCode::ERROR_OVERCUTTING_NOGAPINLIMIT;
		return ErrorCode::ERROR_NONE;
	}
}

ErrorCode OverCuttingDivisionPlanner::Plan(const QList<RS_Entity*>& entityList,
	const OverCuttingDivisionParams& params,
	OverCuttingDivisionResult& result)
{
	result = OverCuttingDivisionResult();
	if (entityList.empty())
		return ErrorCode::ERROR_OVERCUTTING_NOCUTENTITY;
	if (params.endX <= params.startX + EPS)
		return ErrorCode::ERROR_OVERCUTTING_INVALIDRANGE;

	const double capacity = params.maxLength;
	result.capacity = capacity;
	if (capacity <= EPS)
		return ErrorCode::ERROR_OVERCUTTING_OUTOFLIMIT;

	double targetLength = params.suggestedLength;
	if (targetLength <= EPS)
		targetLength = capacity;
	if (targetLength > capacity + EPS)
		return ErrorCode::ERROR_OVERCUTTING_LSUG_TOO_LARGE;

	const double totalLength = params.endX - params.startX;
	// 总长不超过限位、且用户没给出更短的建议段长 → 视为无需切分
	// 注意：若用户显式填了 suggestedLength 且小于 totalLength，仍要继续分段（包括等间距模式）
	const bool userProvidedSuggested = params.suggestedLength > EPS;
	if (totalLength <= capacity + EPS &&
		(!userProvidedSuggested || targetLength >= totalLength - EPS))
	{
		result.effectiveLength = totalLength;
		return ErrorCode::ERROR_NONE;
	}

	const std::vector<Range> occupied = mergeRanges(buildOccupiedRanges(entityList));
	const std::vector<double> endpointCandidates = buildEndpointCandidates(entityList, params.startX, params.endX);
	double segmentLeft = params.startX;
	if (!params.equalLength)
	{
		result.effectiveLength = targetLength;
		// 段长上限取 min(限位容量, 建议段长)，这样图元全在限位内也能按建议长度继续切分
		const double stepCap = std::min(capacity, targetLength);
		while (params.endX - segmentLeft > stepCap + EPS)
		{
			const double minCut = segmentLeft;
			const double maxCut = std::min(segmentLeft + capacity, params.endX);
			const double target = std::min(segmentLeft + targetLength, maxCut);

			double cut = 0.0;
			if (!findDivisionCut(occupied, endpointCandidates, minCut, maxCut, target, cut))
				return ErrorCode::ERROR_OVERCUTTING_NOGAPINLIMIT;
			if (cut <= segmentLeft + EPS)
				return ErrorCode::ERROR_OVERCUTTING_SEGTOOSHORT;

			result.divisions.push_back(cut);
			segmentLeft = cut;
		}

		normalizeDivisions(result.divisions, params.startX, params.endX);
		return validateDivisions(result.divisions, params.startX, params.endX, capacity);
	}

	const int count = static_cast<int>(std::ceil(totalLength / targetLength));
	if (count <= 1)
	{
		result.effectiveLength = totalLength;
		return ErrorCode::ERROR_NONE;
	}

	targetLength = totalLength / count;
	result.effectiveLength = targetLength;
	// 按 count 显式产出 count-1 个均分点，避免误用 capacity 作为循环条件导致漏切
	for (int index = 1; index < count; ++index)
	{
		const double minCut = segmentLeft;
		const double maxCut = std::min(segmentLeft + capacity, params.endX);
		double target = params.startX + targetLength * index;
		if (target <= segmentLeft + EPS)
			target = segmentLeft + targetLength;
		if (target > maxCut)
			target = maxCut;

		double cut = 0.0;
		if (!findDivisionCut(occupied, endpointCandidates, minCut, maxCut, target, cut))
			return ErrorCode::ERROR_OVERCUTTING_NOGAPINLIMIT;
		if (cut <= segmentLeft + EPS)
			return ErrorCode::ERROR_OVERCUTTING_SEGTOOSHORT;

		result.divisions.push_back(cut);
		segmentLeft = cut;
	}

	normalizeDivisions(result.divisions, params.startX, params.endX);
	return validateDivisions(result.divisions, params.startX, params.endX, capacity);
}
