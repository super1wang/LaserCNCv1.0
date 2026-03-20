#include "gui/gui_application.h"
#include "gui/gui_document.h"
#include "base/lcnc_application.h"

// ── Singleton ─────────────────────────────────────────────────────────────────
GuiApplication* GuiApplication::s_instance = nullptr;

GuiApplication* GuiApplication::instance()
{
    if (!s_instance)
        s_instance = new GuiApplication();
    return s_instance;
}

GuiApplication::GuiApplication(QObject* parent)
    : QObject(parent)
{
    LcncApplication* app = LcncApplication::instance();
    connect(app, &LcncApplication::documentAdded,
            this, &GuiApplication::onDocumentAdded);
    connect(app, &LcncApplication::documentClosed,
            this, &GuiApplication::onDocumentClosed);
    connect(app, &LcncApplication::activeDocumentChanged,
            this, &GuiApplication::onActiveDocumentChanged);
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
    return guiDocument(LcncApplication::instance()->activeDocumentId());
}
