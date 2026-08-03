#include "core/algorithms/cad/primitives.h"
#include "modules/cam/services/machining_face_pipeline_service.h"
#include "modules/cam/services/toolpath_generation_service.h"

#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>

#include <cassert>

int main()
{
    lcnc::cam::ToolpathGenerationStamp captured;
    captured.toolpathRevision = 10;
    captured.machiningFaceRevision = 20;
    captured.machineSetupRevision = 30;
    captured.leadInLength = 4.0;
    captured.smoothAngle = 5.0;
    captured.deflection = 0.1;
    captured.useFaceClassification = true;
    captured.extractionStrategy = 2;
    captured.contourIds = {101, 102};
    captured.sources.push_back({
        QStringLiteral("0:1"), 0, lcnc::cad_algo::makeBox(10.0, 10.0, 2.0)});

    auto current = captured;
    using lcnc::cam::ToolpathGenerationService;
    assert(ToolpathGenerationService::acceptsResult(captured, current, true, false));
    assert(!ToolpathGenerationService::acceptsResult(captured, current, false, false));
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, true));

    current = captured;
    ++current.machiningFaceRevision;
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, false));

    current = captured;
    ++current.machineSetupRevision;
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, false));

    current = captured;
    current.leadInLength += 1.0;
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, false));

    current = captured;
    current.smoothAngle += 1.0;
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, false));

    current = captured;
    current.deflection += 0.01;
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, false));

    current = captured;
    current.useFaceClassification = false;
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, false));

    current = captured;
    ++current.extractionStrategy;
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, false));

    current = captured;
    current.contourIds.pop_back();
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, false));

    current = captured;
    current.sources[0].shape = lcnc::cad_algo::makeBox(11.0, 10.0, 2.0);
    assert(!ToolpathGenerationService::acceptsResult(captured, current, true, false));

    lcnc::cam::MachiningFacePipelineService faces;
    lcnc::cam::MachiningFacePipelineService::Entry face;
    face.faceId = faces.nextFaceId();
    face.workpieceEntry = QStringLiteral("0:1");
    const TopoDS_Shape faceSource = lcnc::cad_algo::makeBox(10.0, 10.0, 2.0);
    TopExp_Explorer faceExplorer(faceSource, TopAbs_FACE);
    assert(faceExplorer.More());
    face.face = TopoDS::Face(faceExplorer.Current());
    faces.entries().push_back(face);
    const auto initialRevision = faces.revision();
    assert(initialRevision != 0);
    const auto records = faces.persistenceRecords();
    assert(records.size() == 1);
    assert(records.front().faceId == face.faceId);
    assert(records.front().signature != 0);
    faces.entries().front().manual = true;
    assert(faces.revision() != initialRevision);
    const lcnc::cam::MachiningFacePipelineService::Candidate duplicateCandidate{
        face.face, QStringLiteral("0:1"), lcnc::cam::MachiningFaceRole::MachiningSurface};
    assert(faces.replaceAutomaticFaces({duplicateCandidate}));
    assert(faces.entries().size() == 1);
    assert(faces.entries().front().manual);
    assert(faces.removeFace(face.faceId));
    assert(faces.entries().empty());
    faces.reset();
    assert(faces.entries().empty());
    assert(faces.nextFaceId() == 1);

    return 0;
}
