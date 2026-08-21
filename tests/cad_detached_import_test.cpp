#include "core/document/lcnc_document.h"
#include "core/project/lcnc_project_manager.h"
#include "core/task/task_manager.h"
#include "modules/cad/services/cad_document_io_service.h"

#include <QCoreApplication>
#include <QEvent>
#include <QEventLoop>
#include <QTimer>
#include <iostream>

int main(int argc, char** argv) {
    QCoreApplication application(argc, argv);
    auto projectManager = std::make_unique<lcnc::LcncProjectManager>();
    auto taskManager = std::make_unique<TaskManager>();
    auto service = std::make_unique<lcnc::cad::CadDocumentIoService>(*projectManager, *taskManager);
    auto target = LcncDocument::createStandalone(9001, QStringLiteral("target"));
    if (!target) {
        std::cerr << "unable to create target document\n";
        return 1;
    }

    auto task = service->readImportAsync(QString::fromUtf8(LCNC_CAD_IMPORT_MODEL_PATH));
    if (task.id == kInvalidTaskId || !task.payload || !task.error) {
        std::cerr << "unable to schedule detached import\n";
        return 2;
    }
    bool finishedSuccessfully = false;
    QEventLoop waitLoop;
    QObject::connect(taskManager.get(), &TaskManager::taskFinished, &waitLoop,
                     [&](TaskId finishedId, bool success) {
                         if (finishedId != task.id)
                             return;
                         finishedSuccessfully = success;
                         waitLoop.quit();
                     });
    QTimer::singleShot(60000, &waitLoop, &QEventLoop::quit);
    waitLoop.exec();
    if (!finishedSuccessfully) {
        std::cerr << "detached import failed: " << task.error->toStdString() << '\n';
        return 3;
    }

    if (target->entityLabels(LcncDocument::EntityKind::Workpiece).Length() != 0) {
        std::cerr << "worker mutated target document before commit\n";
        return 4;
    }
    QString error;
    if (!lcnc::cad::CadDocumentIoService::commitImport(target.get(), task.payload, &error)) {
        std::cerr << "commit failed: " << error.toStdString() << '\n';
        return 5;
    }
    if (target->entityLabels(LcncDocument::EntityKind::Workpiece).Length() == 0) {
        std::cerr << "commit did not add geometry\n";
        return 6;
    }
    if (lcnc::cad::CadDocumentIoService::commitImport(nullptr, task.payload, &error) ||
        lcnc::cad::CadDocumentIoService::commitImport(target.get(), {}, &error)) {
        std::cerr << "invalid commit input was accepted\n";
        return 7;
    }
    QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
    service.reset();
    target.reset();
    task.payload.reset();
    taskManager.reset();
    projectManager.reset();
    return 0;
}
