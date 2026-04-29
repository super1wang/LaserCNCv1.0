#pragma once

#include <QString>
#include <QVector>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

#include <vector>

namespace lcnc::cad {

/**
 * @brief Supported work planes for the lightweight CAD sketch session.
 */
enum class SketchPlaneKind {
    XY = 0,
    YZ = 1,
    ZX = 2
};

/**
 * @brief Parameterized sketch profiles available before mouse drawing lands.
 */
enum class SketchProfileKind {
    Rectangle = 0,
    Circle = 1
};

/**
 * @brief Sketch element kinds drivable from the TaskPanel tool palette.
 *
 * Index values are also used by `CadModule` and Ribbon commands to select
 * the active drawing tool.
 */
enum class SketchToolKind {
    None = 0,
    Point = 1,
    Line = 2,
    Arc = 3,
    Circle = 4,
    Rectangle = 5,
    Polygon = 6
};

/**
 * @brief One persisted element inside the active sketch session.
 *
 * `params` interpretation per kind:
 *  - Point:     [x, y]
 *  - Line:      [x1, y1, x2, y2]
 *  - Arc:       [x1, y1, xMid, yMid, x2, y2]
 *  - Circle:    [cx, cy, radius]
 *  - Rectangle: [cx, cy, width, height]
 *  - Polygon:   [cx, cy, radius, sides]
 */
struct SketchElement {
    int id{0};
    SketchToolKind kind{SketchToolKind::None};
    QVector<double> params;
    QString label;
};

/**
 * @brief Feature operation applied to a finished sketch profile.
 */
enum class FeatureKind {
    Extrude = 0,
    Revolve = 1,
    Sweep = 2
};

/**
 * @brief Module-layer state for the first Sketch + Feature modeling workflow.
 *
 * The session owns only transient modeling parameters and OCC preview inputs.
 * It never touches LcncDocument, GuiDocument, Kernel, or UI widgets.
 */
class CadModelingSession
{
public:
    /// Start editing a sketch on the requested construction plane.
    void beginSketch(SketchPlaneKind planeKind);

    /// Replace the active sketch profile parameters.
    void setSketchProfile(SketchProfileKind profileKind,
                          double width,
                          double height,
                          double radius);

    /// Active drawing tool selected from the TaskPanel palette.
    SketchToolKind sketchTool() const;
    /// Switch the active drawing tool; only meaningful while editing a sketch.
    void setSketchTool(SketchToolKind tool);

    /// Append a new sketch element. Returns the assigned id, or -1 on validation failure.
    int addSketchElement(SketchToolKind kind,
                         const QVector<double>& params,
                         QString* errMsg = nullptr);
    /// Remove a previously added element by id.
    bool removeSketchElement(int elementId);
    /// Move a sketch element in the active sketch plane.
    bool moveSketchElement(int elementId, double deltaX, double deltaY, QString* errMsg = nullptr);
    /// Move a specific sketch element handle in the active sketch plane.
    bool moveSketchElementHandle(int elementId,
                                 int handleIndex,
                                 double deltaX,
                                 double deltaY,
                                 QString* errMsg = nullptr);
    /// Clear all transient sketch elements collected during the session.
    void clearSketchElements();
    /// Read-only list of current sketch elements.
    const std::vector<SketchElement>& sketchElements() const;

    /// Finish sketch editing and build the profile face used by features.
    bool finishSketch(QString* errMsg = nullptr);

    /// Reset all transient sketch and feature state.
    void clear();

    /// Returns true while a sketch is being edited.
    bool isSketchEditing() const;

    /// Returns true once a sketch profile face is available for features.
    bool hasFinishedProfile() const;

    /// Build a feature shape from the current finished profile.
    TopoDS_Shape buildFeature(FeatureKind featureKind,
                              double length,
                              double angleDeg,
                              QString* errMsg = nullptr) const;

    /// Build a feature shape from an externally stored record (used by SketchManager).
    static TopoDS_Shape buildFeatureFromRecord(SketchPlaneKind plane,
                                               const TopoDS_Face& profileFace,
                                               FeatureKind featureKind,
                                               double length,
                                               double angleDeg,
                                               QString* errMsg = nullptr);

    /// Plane that was used by the last finished sketch.
    SketchPlaneKind finishedPlane() const { return m_planeKind; }
    /// Profile face produced by the last finished sketch.
    const TopoDS_Face& finishedProfileFace() const { return m_profileFace; }

    /// Human-readable default name for a committed feature.
    QString defaultFeatureName(FeatureKind featureKind) const;

private:
    enum class SessionState {
        Idle,
        EditingSketch,
        FinishedSketch
    };

    /// Build an OCC wire from a `SketchElement`. Returns null on validation failure.
    TopoDS_Wire buildElementWire(const SketchElement& element,
                                 QString* errMsg = nullptr) const;

    SessionState m_state{SessionState::Idle};
    SketchPlaneKind m_planeKind{SketchPlaneKind::XY};
    SketchProfileKind m_profileKind{SketchProfileKind::Rectangle};
    SketchToolKind m_sketchTool{SketchToolKind::None};
    double m_profileWidth{80.0};
    double m_profileHeight{50.0};
    double m_profileRadius{25.0};
    int m_nextElementId{1};
    std::vector<SketchElement> m_elements;
    TopoDS_Face m_profileFace;
};

} // namespace lcnc::cad