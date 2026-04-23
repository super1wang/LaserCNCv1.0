#include "view/gui_application.h"
#include "view/gui_document.h"
#include "core/document/lcnc_application.h"
#include "core/kernel/kernel.h"

// ── Singleton accessor ────────────────────────────────────────────────────────
//   Lifecycle owned by lcnc::Kernel — see kernel.cpp::registerCoreServices.
//   Use lcnc::Kernel::current().guiApp() to access this object.
GuiApplication* GuiApplication::s_instance = nullptr;

GuiApplication::GuiApplication(QObject* parent)
    : QObject(parent)
{
    Q_ASSERT_X(!s_instance, "GuiApplication",
               "second GuiApplication instance — must be Kernel-owned only");
    s_instance = this;

    LcncApplication* app = lcnc::Kernel::current().app();
    connect(app, &LcncApplication::documentAdded,
            this, &GuiApplication::onDocumentAdded);
    connect(app, &LcncApplication::documentClosed,
            this, &GuiApplication::onDocumentClosed);
    connect(app, &LcncApplication::activeDocumentChanged,
            this, &GuiApplication::onActiveDocumentChanged);
}

GuiApplication::~GuiApplication()
{
    if (s_instance == this) s_instance = nullptr;
}

// ── Slots ──────────────────────────────────────────────────────────────────────
void GuiApplication::onDocumentAdded(DocumentId id)
{
    auto* guiDoc = new GuiDocument(id, this);
    m_guiDocs.insert(id, guiDoc);
    emit guiDocumentAdded(id);
}

void GuiApplication::onDocumentClosed(DocumentId id)
{
    if (GuiDocument* gd = m_guiDocs.take(id)) {
        emit guiDocumentClosed(id);
        delete gd;
    }
}

void GuiApplication::onActiveDocumentChanged(DocumentId id)
{
    emit activeGuiDocumentChanged(id);
}

// ── Accessors ──────────────────────────────────────────────────────────────────
GuiDocument* GuiApplication::guiDocument(DocumentId id) const
{
    return m_guiDocs.value(id, nullptr);
}

QList<GuiDocument*> GuiApplication::guiDocuments() const
{
    return m_guiDocs.values();
}

GuiDocument* GuiApplication::activeGuiDocument() const
{
    return guiDocument(lcnc::Kernel::current().app()->activeDocumentId());
}

GuiDocument* GuiApplication::machineGuiDocument() const
{
    return guiDocument(lcnc::Kernel::current().app()->machineDocumentId());
}
