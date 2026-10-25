#include "WeldSeamTransition.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <vector>

#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <Extrema_ExtPC.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <Geom2d_Curve.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>

namespace weld_traj {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDedupMm = 0.02;
constexpr double kCornerJoinMm = 0.1;
constexpr double kGapBridgeMm = 30.0;
constexpr int kMaxSamples = 10000;

struct SeamGeom {
    ProcessedEdge pe;
    TopoDS_Face face1;
    TopoDS_Face face2;
    double originUStart = 0.0;
    double originUEnd = 0.0;
    double uStart = 0.0;
    double uEnd = 0.0;
    bool valid = false;
};

struct TransitionLink {
    bool hasCurve = false;
    TopoDS_Edge curve;
    bool reverse = false;
};

struct FrameMemory {
    bool hasZ = false;
    gp_Dir z = gp_Dir(0.0, 0.0, 1.0);
    bool hasInH = false;
    gp_Vec inH = gp_Vec(1.0, 0.0, 0.0);
    bool hasY = false;
    gp_Dir y = gp_Dir(1.0, 0.0, 0.0);
};

bool CollectTwoFaces(const TopoDS_Shape& shape,
    const TopoDS_Edge& edge,
    TopoDS_Face& face1,
    TopoDS_Face& face2)
{
    if (shape.IsNull() || edge.IsNull()) {
        return false;
    }

    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

    const TopTools_ListOfShape* found = nullptr;
    for (Standard_Integer i = 1; i <= edgeToFaces.Extent(); ++i) {
        if (edgeToFaces.FindKey(i).IsPartner(edge)) {
            found = &edgeToFaces.FindFromIndex(i);
            break;
        }
    }
    if (found == nullptr || found->Extent() != 2) {
        return false;
    }

    TopTools_ListIteratorOfListOfShape it(*found);
    face1 = TopoDS::Face(it.Value());
    it.Next();
    face2 = TopoDS::Face(it.Value());
    return !face1.IsNull() && !face2.IsNull();
}

bool FaceNormalAt(const TopoDS_Face& face,
    const TopoDS_Edge& edge,
    double uOnEdge,
    gp_Dir& normal)
{
    Standard_Real first = 0.0;
    Standard_Real last = 0.0;
    Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, first, last);
    bool reversedPCurve = false;
    if (pcurve.IsNull()) {
        const TopoDS_Edge rev = TopoDS::Edge(edge.Reversed());
        pcurve = BRep_Tool::CurveOnSurface(rev, face, first, last);
        reversedPCurve = true;
    }
    if (pcurve.IsNull() || std::abs(last - first) <= Precision::PConfusion()) {
        return false;
    }

    double u = uOnEdge;
    if (reversedPCurve) {
        u = first + last - uOnEdge;
    }
    u = std::max(first, std::min(last, u));

    const gp_Pnt2d uv = pcurve->Value(u);
    BRepAdaptor_Surface surf(face, Standard_True);
    BRepLProp_SLProps props(surf, uv.X(), uv.Y(), 1, Precision::Confusion());
    if (!props.IsNormalDefined()) {
        return false;
    }

    normal = props.Normal();
    if (face.Orientation() == TopAbs_REVERSED) {
        normal.Reverse();
    }
    return true;
}

bool ProjectPerp(const gp_Vec& v, const gp_Dir& axis, gp_Vec& out)
{
    const gp_Vec a(axis);
    out = v - a * v.Dot(a);
    if (out.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    out.Normalize();
    return true;
}

bool PrepareSeam(const TopoDS_Shape& shape, const ProcessedEdge& pe, SeamGeom& seam)
{
    seam = SeamGeom();
    seam.pe = pe;
    if (pe.Edge.IsNull() || BRep_Tool::Degenerated(pe.Edge)) {
        return false;
    }
    if (!CollectTwoFaces(shape, pe.Edge, seam.face1, seam.face2)) {
        return false;
    }

    BRepAdaptor_Curve curve(pe.Edge);
    const double u0 = curve.FirstParameter();
    const double u1 = curve.LastParameter();
    if (std::abs(u1 - u0) <= Precision::PConfusion()) {
        return false;
    }

    const gp_Pnt p0 = curve.Value(u0);
    const gp_Pnt p1 = curve.Value(u1);
    if (p0.Distance(pe.StartPoint) <= p1.Distance(pe.StartPoint)) {
        seam.originUStart = u0;
        seam.originUEnd = u1;
    }
    else {
        seam.originUStart = u1;
        seam.originUEnd = u0;
    }
    seam.uStart = seam.originUStart;
    seam.uEnd = seam.originUEnd;
    seam.valid = true;
    return true;
}

double TravelFraction(double uStart, double uEnd, double u)
{
    const double span = uEnd - uStart;
    if (std::abs(span) <= Precision::PConfusion()) {
        return 0.0;
    }
    return (u - uStart) / span;
}

double CurveLength(const TopoDS_Edge& edge, double uA, double uB)
{
    BRepAdaptor_Curve curve(edge);
    return std::abs(GCPnts_AbscissaPoint::Length(curve, uA, uB));
}

bool ProjectParam(const TopoDS_Edge& edge, const gp_Pnt& point, double& u, double& dist)
{
    if (edge.IsNull() || BRep_Tool::Degenerated(edge)) {
        return false;
    }
    BRepAdaptor_Curve curve(edge);
    const double u0 = std::min(curve.FirstParameter(), curve.LastParameter());
    const double u1 = std::max(curve.FirstParameter(), curve.LastParameter());
    if (u1 - u0 <= Precision::PConfusion()) {
        return false;
    }

    Extrema_ExtPC extrema(point, curve, u0, u1);
    if (!extrema.IsDone() || extrema.NbExt() < 1) {
        return false;
    }

    double bestSq = std::numeric_limits<double>::max();
    bool found = false;
    for (Standard_Integer i = 1; i <= extrema.NbExt(); ++i) {
        const double sq = extrema.SquareDistance(i);
        if (sq < bestSq) {
            bestSq = sq;
            u = extrema.Point(i).Parameter();
            found = true;
        }
    }
    if (!found) {
        return false;
    }
    dist = std::sqrt(std::max(0.0, bestSq));
    return true;
}

void EdgeEnds(const TopoDS_Edge& edge, gp_Pnt& first, gp_Pnt& last)
{
    BRepAdaptor_Curve curve(edge);
    first = curve.Value(curve.FirstParameter());
    last = curve.Value(curve.LastParameter());
}

bool TangentAlong(const TopoDS_Edge& edge, double u, double uToward, gp_Pnt& point, gp_Dir& yDir)
{
    BRepAdaptor_Curve curve(edge);
    gp_Vec d1;
    curve.D1(u, point, d1);
    const double step = (uToward - u) * 0.05;
    if (std::abs(step) > Precision::PConfusion()) {
        const gp_Vec hint(point, curve.Value(u + step));
        if (hint.Magnitude() > Precision::Confusion() && d1.Magnitude() > Precision::Confusion()
            && d1.Dot(hint) < 0.0) {
            d1.Reverse();
        }
        if (d1.Magnitude() <= Precision::Confusion()) {
            d1 = hint;
        }
    }
    if (d1.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    yDir = gp_Dir(d1);
    return true;
}

bool TrimToPoint(SeamGeom& seam, const gp_Pnt& point, bool trimEnd, double maxProjectDist)
{
    double u = 0.0;
    double dist = 0.0;
    if (!ProjectParam(seam.pe.Edge, point, u, dist) || dist > maxProjectDist) {
        return false;
    }
    const double fraction = TravelFraction(seam.originUStart, seam.originUEnd, u);
    const double startF = TravelFraction(seam.originUStart, seam.originUEnd, seam.uStart);
    const double endF = TravelFraction(seam.originUStart, seam.originUEnd, seam.uEnd);
    if (trimEnd) {
        if (!(fraction > startF + 1.0e-4 && fraction < endF - 1.0e-6)) {
            return false;
        }
        const double kept = std::abs(fraction - startF);
        if (kept <= 1.0e-4) {
            return false;
        }
        seam.uEnd = u;
        return true;
    }
    if (!(fraction > startF + 1.0e-6 && fraction < endF - 1.0e-4)) {
        return false;
    }
    seam.uStart = u;
    return true;
}

int PointCount(double length, double spacing, int minimum)
{
    if (!(length > Precision::Confusion()) || !(spacing > Precision::Confusion())) {
        return 0;
    }
    int count = static_cast<int>(std::ceil(length / spacing)) + 1;
    if (count < minimum) {
        count = minimum;
    }
    if (count < 2) {
        count = 2;
    }
    if (count > kMaxSamples) {
        count = kMaxSamples;
    }
    return count;
}

bool SampleParameters(const TopoDS_Edge& edge,
    double uTravelStart,
    double uTravelEnd,
    int count,
    std::vector<double>& params)
{
    params.clear();
    if (count < 2 || edge.IsNull()) {
        return false;
    }
    BRepAdaptor_Curve curve(edge);
    const double u0 = std::min(uTravelStart, uTravelEnd);
    const double u1 = std::max(uTravelStart, uTravelEnd);
    if (u1 - u0 <= Precision::PConfusion()) {
        return false;
    }

    GCPnts_UniformAbscissa sampler(curve, count, u0, u1, Precision::Confusion());
    if (sampler.IsDone() && sampler.NbPoints() >= 2) {
        params.reserve(static_cast<std::size_t>(sampler.NbPoints()));
        for (Standard_Integer i = 1; i <= sampler.NbPoints(); ++i) {
            params.push_back(sampler.Parameter(i));
        }
    }
    else {
        params.push_back(u0);
        params.push_back(u1);
    }
    if (uTravelEnd < uTravelStart) {
        std::reverse(params.begin(), params.end());
    }
    return params.size() >= 2;
}

bool TravelTangent(const BRepAdaptor_Curve& curve,
    const std::vector<double>& params,
    std::size_t index,
    gp_Pnt& point,
    gp_Dir& yDir)
{
    gp_Vec d1;
    curve.D1(params[index], point, d1);

    gp_Vec hint;
    bool hasHint = false;
    if (index + 1 < params.size()) {
        hint = gp_Vec(point, curve.Value(params[index + 1]));
        hasHint = hint.Magnitude() > Precision::Confusion();
    }
    else if (index > 0) {
        hint = gp_Vec(curve.Value(params[index - 1]), point);
        hasHint = hint.Magnitude() > Precision::Confusion();
    }

    if (d1.Magnitude() <= Precision::Confusion()) {
        if (!hasHint) {
            return false;
        }
        d1 = hint;
    }
    else if (hasHint && d1.Dot(hint) < 0.0) {
        d1.Reverse();
    }
    if (d1.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    yDir = gp_Dir(d1);
    return true;
}

bool FinishFrame(const gp_Pnt& position, const gp_Dir& yDir, const gp_Dir& zIn, DiscretePoint& sample)
{
    gp_Vec xVec = gp_Vec(yDir).Crossed(gp_Vec(zIn));
    if (xVec.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    xVec.Normalize();
    gp_Vec zVec = xVec.Crossed(gp_Vec(yDir));
    if (zVec.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    if (zVec.Dot(gp_Vec(zIn)) < 0.0) {
        xVec.Reverse();
        zVec.Reverse();
    }
    zVec.Normalize();
    sample.position = position;
    sample.xDir = gp_Dir(xVec);
    sample.yDir = yDir;
    sample.zDir = gp_Dir(zVec);
    return true;
}

bool InwardZ(const SeamGeom& seam, double u, const gp_Dir& yDir, gp_Dir& zDir)
{
    gp_Dir n1;
    gp_Dir n2;
    if (!FaceNormalAt(seam.face1, seam.pe.Edge, u, n1)
        || !FaceNormalAt(seam.face2, seam.pe.Edge, u, n2)) {
        return false;
    }

    gp_Vec p1;
    gp_Vec p2;
    if (!ProjectPerp(gp_Vec(n1), yDir, p1) || !ProjectPerp(gp_Vec(n2), yDir, p2)) {
        return false;
    }

    gp_Vec bisector = -(p1 + p2);
    if (bisector.Magnitude() <= Precision::Confusion()) {
        bisector = -p1;
    }
    if (bisector.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    zDir = gp_Dir(bisector);
    return true;
}

bool BuildFlatSample(const SeamGeom& seam,
    double u,
    const gp_Dir& yDir,
    const gp_Pnt& point,
    const gp_Dir* prevZ,
    DiscretePoint& sample)
{
    gp_Dir zDir;
    if (!InwardZ(seam, u, yDir, zDir)) {
        return false;
    }
    if (prevZ != nullptr && zDir.Dot(*prevZ) < 0.0) {
        zDir.Reverse();
    }
    return FinishFrame(point, yDir, zDir, sample);
}

bool HorizontalInward(const SeamGeom& seam, double u, const gp_Vec* prevInH, gp_Vec& inH)
{
    gp_Dir n1;
    gp_Dir n2;
    if (!FaceNormalAt(seam.face1, seam.pe.Edge, u, n1)
        || !FaceNormalAt(seam.face2, seam.pe.Edge, u, n2)) {
        return false;
    }

    // 两侧面法向夹角平分线 n1+n2 指向焊缝内侧。
    // 取反会把立焊 Z 的水平分量带到焊缝外侧。
    const gp_Vec inward = gp_Vec(n1) + gp_Vec(n2);
    inH = gp_Vec(inward.X(), inward.Y(), 0.0);
    if (inH.Magnitude() <= Precision::Confusion()) {
        if (prevInH != nullptr && prevInH->Magnitude() > Precision::Confusion()) {
            inH = *prevInH;
        }
        else {
            inH = gp_Vec(1.0, 0.0, 0.0);
        }
    }
    inH.Normalize();
    if (prevInH != nullptr && inH.Dot(*prevInH) < 0.0) {
        inH.Reverse();
    }
    return true;
}

bool BuildVerticalSample(const SeamGeom& seam,
    double u,
    const gp_Pnt& point,
    double tiltRad,
    const gp_Vec* prevInH,
    const gp_Dir* prevY,
    gp_Vec& outInH,
    DiscretePoint& sample)
{
    gp_Vec inH;
    if (!HorizontalInward(seam, u, prevInH, inH)) {
        return false;
    }

    const gp_Vec zVec = inH * std::cos(tiltRad) + gp_Vec(0.0, 0.0, 1.0) * std::sin(tiltRad);
    if (zVec.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    const gp_Dir zDir(zVec);

    gp_Vec yVec(-inH.Y(), inH.X(), 0.0);
    if (yVec.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    if (prevY != nullptr && yVec.Dot(gp_Vec(*prevY)) < 0.0) {
        yVec.Reverse();
    }
    const gp_Dir yDir(yVec);
    if (!FinishFrame(point, yDir, zDir, sample)) {
        return false;
    }
    outInH = inH;
    return true;
}

bool BlendZ(const gp_Dir& zFrom, const gp_Dir& zTo, double t, const gp_Dir& yDir, gp_Dir& zOut)
{
    gp_Dir to = zTo;
    if (to.Dot(zFrom) < 0.0) {
        to.Reverse();
    }
    gp_Vec z = gp_Vec(zFrom) * (1.0 - t) + gp_Vec(to) * t;
    z -= gp_Vec(yDir) * z.Dot(gp_Vec(yDir));
    if (z.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    zOut = gp_Dir(z);
    return true;
}

bool AppendPoint(std::vector<DiscretePoint>& points, const DiscretePoint& sample)
{
    if (!points.empty() && points.back().position.Distance(sample.position) <= kDedupMm) {
        return false;
    }
    points.push_back(sample);
    return true;
}

void Remember(FrameMemory& memory, const DiscretePoint& sample, const gp_Vec* inH)
{
    memory.z = sample.zDir;
    memory.hasZ = true;
    memory.y = sample.yDir;
    memory.hasY = true;
    if (inH != nullptr) {
        memory.inH = *inH;
        memory.hasInH = true;
    }
}

DiscretePoint RetractPoint(const DiscretePoint& seam, double retractMm)
{
    DiscretePoint point = seam;
    point.position = seam.position.Translated(gp_Vec(seam.zDir) * (-retractMm));
    return point;
}

}  // namespace
}  // namespace weld_traj

bool WeldingPathBuilder::BuildClusterTrajectory(const TopoDS_Shape& selectShape,
    const EdgeCluster& cluster,
    double spacingMm,
    double retractMm,
    double verticalTiltDeg,
    std::vector<DiscretePoint>& out)
{
    using namespace weld_traj;

    out.clear();
    if (cluster.edges.empty()) {
        return false;
    }

    const bool vertical = cluster.isVertical;
    const double tiltRad = verticalTiltDeg * kPi / 180.0;
    const double maxProjectDist = std::max(1.0, m_standoff);
    const double cornerTol = std::max(m_tol, kCornerJoinMm);

    std::vector<SeamGeom> seams;
    seams.reserve(cluster.edges.size());
    for (const ProcessedEdge& edge : cluster.edges) {
        SeamGeom seam;
        if (!PrepareSeam(selectShape, edge, seam)) {
            return false;
        }
        seams.push_back(seam);
    }

    std::vector<TransitionLink> links(seams.size() > 0 ? seams.size() - 1 : 0);
    for (std::size_t i = 0; i + 1 < seams.size(); ++i) {
        const ProcessedEdge& prev = seams[i].pe;
        const ProcessedEdge& next = seams[i + 1].pe;
        const double gap = prev.EndPoint.Distance(next.StartPoint);

        TopoDS_Edge transition;
        bool fillet = false;
        if (gap <= cornerTol && BuildFilletArc(prev, next, transition) && !transition.IsNull()) {
            gp_Pnt first;
            gp_Pnt last;
            EdgeEnds(transition, first, last);
            const double forward = first.Distance(prev.EndPoint) + last.Distance(next.StartPoint);
            const double backward = last.Distance(prev.EndPoint) + first.Distance(next.StartPoint);
            const bool reverse = backward < forward;
            const gp_Pnt from = reverse ? last : first;
            const gp_Pnt to = reverse ? first : last;

            SeamGeom savedPrev = seams[i];
            SeamGeom savedNext = seams[i + 1];
            const bool trimmed = TrimToPoint(seams[i], from, true, maxProjectDist)
                && TrimToPoint(seams[i + 1], to, false, maxProjectDist);
            if (trimmed) {
                links[i].hasCurve = true;
                links[i].curve = transition;
                links[i].reverse = reverse;
                fillet = true;
            }
            else {
                seams[i] = savedPrev;
                seams[i + 1] = savedNext;
            }
        }

        if (!fillet && gap < kGapBridgeMm
            && BuildInterpolationCurve(prev, next, transition) && !transition.IsNull()) {
            BRepAdaptor_Curve bridge(transition);
            const double bridgeLen = std::abs(GCPnts_AbscissaPoint::Length(
                bridge, bridge.FirstParameter(), bridge.LastParameter()));
            if (bridgeLen > Precision::Confusion()) {
                gp_Pnt first;
                gp_Pnt last;
                EdgeEnds(transition, first, last);
                const double forward = first.Distance(prev.EndPoint) + last.Distance(next.StartPoint);
                const double backward = last.Distance(prev.EndPoint) + first.Distance(next.StartPoint);
                links[i].hasCurve = true;
                links[i].curve = transition;
                links[i].reverse = backward < forward;
            }
        }
    }

    std::vector<DiscretePoint> seamPoints;
    FrameMemory memory;

    auto sampleSeam = [&](const SeamGeom& seam) -> bool {
        const double length = CurveLength(seam.pe.Edge, seam.uStart, seam.uEnd);
        if (length <= Precision::Confusion()) {
            return true;
        }
        const int count = PointCount(length, spacingMm, 2);
        std::vector<double> params;
        if (!SampleParameters(seam.pe.Edge, seam.uStart, seam.uEnd, count, params)) {
            return false;
        }

        BRepAdaptor_Curve curve(seam.pe.Edge);
        for (std::size_t index = 0; index < params.size(); ++index) {
            gp_Pnt point;
            gp_Dir travel;
            if (!TravelTangent(curve, params, index, point, travel)) {
                return false;
            }

            DiscretePoint sample;
            gp_Vec inH;
            bool built = false;
            if (vertical) {
                built = BuildVerticalSample(seam,
                    params[index],
                    point,
                    tiltRad,
                    memory.hasInH ? &memory.inH : nullptr,
                    memory.hasY ? &memory.y : nullptr,
                    inH,
                    sample);
            }
            else {
                built = BuildFlatSample(seam,
                    params[index],
                    travel,
                    point,
                    memory.hasZ ? &memory.z : nullptr,
                    sample);
            }
            if (!built) {
                return false;
            }
            if (AppendPoint(seamPoints, sample)) {
                Remember(memory, sample, vertical ? &inH : nullptr);
            }
        }
        return true;
    };

    auto sampleFlatTransition = [&](const TransitionLink& link, const SeamGeom& nextSeam) -> bool {
        std::vector<double> params;
        BRepAdaptor_Curve curve(link.curve);
        const double u0 = curve.FirstParameter();
        const double u1 = curve.LastParameter();
        const double length = std::abs(GCPnts_AbscissaPoint::Length(curve, u0, u1));
        const double step = std::min(spacingMm, std::max(m_standoff, 0.5));
        const int count = PointCount(std::max(length, step), step, 5);
        const double travelStart = link.reverse ? u1 : u0;
        const double travelEnd = link.reverse ? u0 : u1;
        if (!SampleParameters(link.curve, travelStart, travelEnd, count, params)) {
            return false;
        }

        gp_Pnt nextPoint;
        gp_Dir nextTravel;
        const bool hasNextTravel = TangentAlong(nextSeam.pe.Edge,
            nextSeam.uStart,
            nextSeam.uEnd,
            nextPoint,
            nextTravel);
        DiscretePoint preview;
        const bool hasPreview = hasNextTravel && BuildFlatSample(nextSeam,
            nextSeam.uStart,
            nextTravel,
            nextPoint,
            nullptr,
            preview);
        gp_Dir zFrom = memory.hasZ ? memory.z : (hasPreview ? preview.zDir : gp_Dir(0.0, 0.0, -1.0));
        gp_Dir zTo = hasPreview ? preview.zDir : zFrom;
        if (zTo.Dot(zFrom) < 0.0) {
            zTo.Reverse();
        }

        if (!seamPoints.empty()) {
            const gp_Pnt arcStart = curve.Value(params.front());
            if (seamPoints.back().position.Distance(arcStart) <= maxProjectDist) {
                seamPoints.back().position = arcStart;
            }
        }

        for (std::size_t index = 0; index < params.size(); ++index) {
            gp_Pnt point;
            gp_Dir yDir;
            if (!TravelTangent(curve, params, index, point, yDir)) {
                return false;
            }
            const double t = params.size() == 1
                ? 0.0
                : static_cast<double>(index) / static_cast<double>(params.size() - 1);
            gp_Dir zDir;
            if (!BlendZ(zFrom, zTo, t, yDir, zDir)) {
                return false;
            }
            if (memory.hasZ && zDir.Dot(memory.z) < 0.0) {
                zDir.Reverse();
            }
            DiscretePoint sample;
            if (!FinishFrame(point, yDir, zDir, sample)) {
                return false;
            }
            if (AppendPoint(seamPoints, sample)) {
                Remember(memory, sample, nullptr);
            }
        }
        return true;
    };

    auto sampleVerticalTransition = [&](const TransitionLink& link, const SeamGeom& nextSeam) -> bool {
        BRepAdaptor_Curve curve(link.curve);
        const double u0 = curve.FirstParameter();
        const double u1 = curve.LastParameter();
        const double length = std::abs(GCPnts_AbscissaPoint::Length(curve, u0, u1));
        const double step = std::min(spacingMm, std::max(m_standoff, 0.5));
        const int count = PointCount(std::max(length, step), step, 5);
        const double travelStart = link.reverse ? u1 : u0;
        const double travelEnd = link.reverse ? u0 : u1;
        std::vector<double> params;
        if (!SampleParameters(link.curve, travelStart, travelEnd, count, params)) {
            return false;
        }

        gp_Vec inFrom = memory.hasInH ? memory.inH : gp_Vec(1.0, 0.0, 0.0);
        if (inFrom.Magnitude() <= Precision::Confusion()) {
            inFrom = gp_Vec(1.0, 0.0, 0.0);
        }
        inFrom.Normalize();
        const gp_Dir yFrom = memory.hasY ? memory.y : gp_Dir(-inFrom.Y(), inFrom.X(), 0.0);

        BRepAdaptor_Curve nextCurve(nextSeam.pe.Edge);
        const gp_Pnt nextPoint = nextCurve.Value(nextSeam.uStart);
        DiscretePoint nextFrame;
        gp_Vec inTo;
        if (!BuildVerticalSample(nextSeam,
                nextSeam.uStart,
                nextPoint,
                tiltRad,
                &inFrom,
                &yFrom,
                inTo,
                nextFrame)) {
            return false;
        }
        if (inTo.Dot(inFrom) < 0.0) {
            inTo.Reverse();
        }

        for (std::size_t index = 0; index < params.size(); ++index) {
            const double t = params.size() == 1
                ? 0.0
                : static_cast<double>(index) / static_cast<double>(params.size() - 1);
            const gp_Pnt point = curve.Value(params[index]);
            gp_Vec inH = inFrom * (1.0 - t) + inTo * t;
            inH.SetZ(0.0);
            if (inH.Magnitude() <= Precision::Confusion()) {
                return false;
            }
            inH.Normalize();
            const gp_Vec zVec = inH * std::cos(tiltRad) + gp_Vec(0.0, 0.0, 1.0) * std::sin(tiltRad);
            gp_Vec yVec(-inH.Y(), inH.X(), 0.0);
            if (yVec.Dot(gp_Vec(yFrom)) < 0.0) {
                yVec.Reverse();
            }
            if (yVec.Magnitude() <= Precision::Confusion() || zVec.Magnitude() <= Precision::Confusion()) {
                return false;
            }
            DiscretePoint sample;
            if (!FinishFrame(point, gp_Dir(yVec), gp_Dir(zVec), sample)) {
                return false;
            }
            if (AppendPoint(seamPoints, sample)) {
                Remember(memory, sample, &inH);
            }
        }
        return true;
    };

    for (std::size_t i = 0; i < seams.size(); ++i) {
        if (!sampleSeam(seams[i])) {
            return false;
        }
        if (i >= links.size() || !links[i].hasCurve) {
            continue;
        }
        const bool linked = vertical
            ? sampleVerticalTransition(links[i], seams[i + 1])
            : sampleFlatTransition(links[i], seams[i + 1]);
        if (!linked) {
            return false;
        }
    }

    if (seamPoints.empty()) {
        return false;
    }

    if (retractMm > Precision::Confusion()) {
        out.push_back(RetractPoint(seamPoints.front(), retractMm));
    }
    out.insert(out.end(), seamPoints.begin(), seamPoints.end());
    if (retractMm > Precision::Confusion()) {
        out.push_back(RetractPoint(seamPoints.back(), retractMm));
    }
    return true;
}

bool WeldingPathBuilder::BuildWeldingTrajectory(const TopoDS_Shape& selectShape,
    const std::vector<EdgeCluster>& sortedClusters,
    std::vector<DiscretePoint>& trajectory,
    double spacingMm,
    double retractMm,
    double verticalTiltDeg)
{
    trajectory.clear();
    if (selectShape.IsNull() || sortedClusters.empty()) {
        return false;
    }
    if (!(spacingMm > Precision::Confusion()) || retractMm < 0.0) {
        return false;
    }
    if (!(verticalTiltDeg > 1.0 && verticalTiltDeg < 89.0)) {
        return false;
    }

    try {
        std::vector<DiscretePoint> merged;
        for (const EdgeCluster& cluster : sortedClusters) {
            if (cluster.edges.empty()) {
                continue;
            }
            std::vector<DiscretePoint> oneCluster;
            if (!BuildClusterTrajectory(selectShape,
                    cluster,
                    spacingMm,
                    retractMm,
                    verticalTiltDeg,
                    oneCluster)) {
                trajectory.clear();
                return false;
            }
            merged.insert(merged.end(), oneCluster.begin(), oneCluster.end());
        }
        if (merged.empty()) {
            return false;
        }
        trajectory.swap(merged);
        return true;
    }
    catch (const std::exception&) {
        trajectory.clear();
        return false;
    }
}

// ---------------------------------------------------------------------------
// 原 WeldCoordinateSystem.cpp（单条边坐标系 / 离散）。
// 焊枪整条轨迹使用 WeldingPathBuilder::BuildWeldingTrajectory。
// ---------------------------------------------------------------------------
#include <TopExp.hxx>

namespace {

    //! Align @p tangent with the topological edge sense FirstVertex → LastVertex.
    //! BRepAdaptor_Curve::D1 can come out anti-parallel to that sense; flip if needed.
    void AlignTangentWithEdgeOrientation(const TopoDS_Edge& edge, gp_Dir& tangent)
    {
        const TopoDS_Vertex vFirst = TopExp::FirstVertex(edge, Standard_True);
        const TopoDS_Vertex vLast = TopExp::LastVertex(edge, Standard_True);
        if (vFirst.IsNull() || vLast.IsNull()) {
            return;
        }

        const gp_Vec chord(BRep_Tool::Pnt(vFirst), BRep_Tool::Pnt(vLast));
        if (chord.Magnitude() <= Precision::Confusion()) {
            return;
        }

        if (tangent.Dot(gp_Dir(chord)) > 0.0) {
            tangent.Reverse();
        }
    }

    //! Mid-parameter point and unit tangent of @p edge, matching edge orientation.
    Standard_Boolean EdgeFrameSeed(const TopoDS_Edge& edge,
        gp_Pnt& origin,
        gp_Dir& yDir)
    {
        if (edge.IsNull() || BRep_Tool::Degenerated(edge)) {
            return Standard_False;
        }

        BRepAdaptor_Curve curve(edge);
        const Standard_Real uFirst = curve.FirstParameter();
        const Standard_Real uLast = curve.LastParameter();
        if (Abs(uLast - uFirst) <= Precision::PConfusion()) {
            return Standard_False;
        }

        const Standard_Real uMid = 0.5 * (uFirst + uLast);
        gp_Pnt p;
        gp_Vec d1;
        curve.D1(uMid, p, d1);

        if (d1.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }

        origin = p;
        yDir = gp_Dir(d1);
        AlignTangentWithEdgeOrientation(edge, yDir);
        return Standard_True;
    }

    //! Face normal at edge parameter @p uOnEdge (same domain as BRepAdaptor_Curve).
    Standard_Boolean FaceNormalAtEdgeParam(const TopoDS_Face& face,
        const TopoDS_Edge& edge,
        Standard_Real      uOnEdge,
        gp_Dir& normal)
    {
        Standard_Real first = 0.0;
        Standard_Real last = 0.0;
        Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, first, last);
        Standard_Boolean     reversedPCurve = Standard_False;

        if (pcurve.IsNull()) {
            const TopoDS_Edge rev = TopoDS::Edge(edge.Reversed());
            pcurve = BRep_Tool::CurveOnSurface(rev, face, first, last);
            reversedPCurve = Standard_True;
        }

        if (pcurve.IsNull() || Abs(last - first) <= Precision::PConfusion()) {
            return Standard_False;
        }

        // Map 3D-edge parameter onto the pcurve domain when the reversed edge was used.
        Standard_Real u = uOnEdge;
        if (reversedPCurve) {
            u = first + last - uOnEdge;
        }
        u = std::max(first, std::min(last, u));

        const gp_Pnt2d uv = pcurve->Value(u);

        BRepAdaptor_Surface surf(face, Standard_True);
        BRepLProp_SLProps   props(surf, uv.X(), uv.Y(), 1, Precision::Confusion());
        if (!props.IsNormalDefined()) {
            return Standard_False;
        }

        normal = props.Normal();
        if (face.Orientation() == TopAbs_REVERSED) {
            normal.Reverse();
        }
        return Standard_True;
    }

    Standard_Boolean FaceNormalAtEdge(const TopoDS_Face& face,
        const TopoDS_Edge& edge,
        gp_Dir& normal)
    {
        Standard_Real f = 0.0;
        Standard_Real l = 0.0;
        BRep_Tool::Range(edge, f, l);
        return FaceNormalAtEdgeParam(face, edge, 0.5 * (f + l), normal);
    }

    void CollectAdjacentFaces(const TopoDS_Shape& shape,
        const TopoDS_Edge& edge,
        TopTools_ListOfShape& faces)
    {
        faces.Clear();

        TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

        for (Standard_Integer i = 1; i <= edgeToFaces.Extent(); ++i) {
            if (edgeToFaces.FindKey(i).IsPartner(edge)) {
                faces = edgeToFaces.FindFromIndex(i);
                return;
            }
        }
    }

    //! Project @p v onto the plane perpendicular to unit direction @p axis.
    Standard_Boolean ProjectPerp(const gp_Vec& v, const gp_Dir& axis, gp_Vec& out)
    {
        const gp_Vec a(axis);
        out = v - v.Dot(a) * a;
        if (out.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }
        out.Normalize();
        return Standard_True;
    }

    //! Unit dihedral bisector of the two face normals at parameter @p u, ⊥ @p yDir.
    //! Uses projected unit normals so Z lies exactly on the angle bisector in the
    //! plane perpendicular to the edge.
    Standard_Boolean BisectorAtParam(const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const TopoDS_Edge& edge,
        Standard_Real      u,
        const gp_Dir& yDir,
        gp_Dir& zDir)
    {
        gp_Dir n1;
        gp_Dir n2;
        if (!FaceNormalAtEdgeParam(face1, edge, u, n1)
            || !FaceNormalAtEdgeParam(face2, edge, u, n2)) {
            return Standard_False;
        }

        gp_Vec p1;
        gp_Vec p2;
        if (!ProjectPerp(gp_Vec(n1), yDir, p1) || !ProjectPerp(gp_Vec(n2), yDir, p2)) {
            return Standard_False;
        }

        gp_Vec bisector = p1 + p2;
        if (bisector.Magnitude() <= Precision::Confusion()) {
            // Faces nearly coplanar / opposite: fall back to either projected normal.
            bisector = p1;
        }

        if (bisector.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }

        zDir = gp_Dir(bisector);
        return Standard_True;
    }

    //! Build one sample: Y along edge, Z = ± face-normal bisector, X = Y × Z.
    Standard_Boolean BuildDiscretePointAt(const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const TopoDS_Edge& edge,
        const BRepAdaptor_Curve& curve,
        Standard_Real            u,
        Standard_Boolean         reverseZ,
        const gp_Dir* prevZ,
        DiscretePoint& sample)
    {
        gp_Pnt p;
        gp_Vec d1;
        curve.D1(u, p, d1);
        if (d1.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }

        gp_Dir yDir(d1); // travel / edge orientation
        // 保证 Y 与 TopoDS_Edge 拓扑方向（First→Last）一致，而不是其反方向
        AlignTangentWithEdgeOrientation(edge, yDir);

        gp_Dir zDir;
        if (!BisectorAtParam(face1, face2, edge, u, yDir, zDir)) {
            return Standard_False;
        }

        // reverseZ=True → opposite bisector (二分角反向).
        if (reverseZ) {
            zDir.Reverse();
        }

        // Keep Z continuous along the path (avoids 180° flips / fan artifacts).
        if (prevZ != nullptr && zDir.Dot(*prevZ) < 0.0) {
            zDir.Reverse();
        }

        // Right-hand rule: X = Y × Z  ⇒  X × Y = Z.
        gp_Vec xVec = gp_Vec(yDir).Crossed(gp_Vec(zDir));
        if (xVec.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }

        sample.position = p;
        sample.yDir = yDir;
        sample.zDir = zDir;
        sample.xDir = gp_Dir(xVec);
        return Standard_True;
    }

    Standard_Boolean BuildFrameFromFaces(const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const TopoDS_Edge& edge,
        const gp_Pnt& origin,
        const gp_Dir& yDir,
        Standard_Boolean   reverseZ,
        gp_Ax3& weldAxis)
    {
        Standard_Real f = 0.0;
        Standard_Real l = 0.0;
        BRep_Tool::Range(edge, f, l);

        gp_Dir zDir;
        if (!BisectorAtParam(face1, face2, edge, 0.5 * (f + l), yDir, zDir)) {
            return Standard_False;
        }

        if (reverseZ) {
            zDir.Reverse();
        }

        gp_Vec xVec = gp_Vec(yDir).Crossed(gp_Vec(zDir));
        if (xVec.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }
        gp_Dir xDir(xVec);

        // gp_Ax3(P, N=Z, Vx=X) builds Y as Z × X, matching +edge when X = Y × Z.
        weldAxis = gp_Ax3(origin, zDir, xDir);
        if (weldAxis.YDirection().Dot(yDir) < 0.0) {
            xDir.Reverse();
            weldAxis = gp_Ax3(origin, zDir, xDir);
        }
        return Standard_True;
    }
} // namespace

Standard_Boolean ComputeWeldCoordinateSystem(const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    gp_Ax3& weldAxis,
    Standard_Boolean    reverseZ)
{
    if (selectShape.IsNull() || edge.IsNull()) {
        return Standard_False;
    }

    TopTools_ListOfShape faceList;
    CollectAdjacentFaces(selectShape, edge, faceList);
    if (faceList.Extent() != 2) {
        return Standard_False;
    }

    TopTools_ListIteratorOfListOfShape it(faceList);
    const TopoDS_Face face1 = TopoDS::Face(it.Value());
    it.Next();
    const TopoDS_Face face2 = TopoDS::Face(it.Value());

    gp_Pnt origin;
    gp_Dir yDir;
    if (!EdgeFrameSeed(edge, origin, yDir)) {
        return Standard_False;
    }

    return BuildFrameFromFaces(face1, face2, edge, origin, yDir, reverseZ, weldAxis);
}

Standard_Boolean DiscretizeWeldTrajectory(const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    std::vector<DiscretePoint>& trajectory,
    Standard_Real               spacing,
    Standard_Boolean            reverseZ,
    Standard_Real               retract)
{
    trajectory.clear();

    if (selectShape.IsNull() || edge.IsNull() || spacing <= Precision::Confusion()) {
        return Standard_False;
    }
    if (BRep_Tool::Degenerated(edge)) {
        return Standard_False;
    }

    TopTools_ListOfShape faceList;
    CollectAdjacentFaces(selectShape, edge, faceList);
    if (faceList.Extent() != 2) {
        return Standard_False;
    }

    TopTools_ListIteratorOfListOfShape it(faceList);
    const TopoDS_Face face1 = TopoDS::Face(it.Value());
    it.Next();
    const TopoDS_Face face2 = TopoDS::Face(it.Value());

    BRepAdaptor_Curve curve(edge);
    const Standard_Real length =
        GCPnts_AbscissaPoint::Length(curve, curve.FirstParameter(), curve.LastParameter());
    if (length <= Precision::Confusion()) {
        return Standard_False;
    }

    GCPnts_UniformAbscissa sampler(curve, spacing);
    if (!sampler.IsDone() || sampler.NbPoints() < 1) {
        return Standard_False;
    }

    trajectory.reserve(static_cast<std::size_t>(sampler.NbPoints()) + 2);
    const gp_Dir* prevZ = nullptr;
    gp_Dir        lastZ;

    for (Standard_Integer i = 1; i <= sampler.NbPoints(); ++i) {
        DiscretePoint sample;
        if (!BuildDiscretePointAt(face1,
            face2,
            edge,
            curve,
            sampler.Parameter(i),
            reverseZ,
            prevZ,
            sample)) {
            trajectory.clear();
            return Standard_False;
        }
        trajectory.push_back(sample);
        lastZ = sample.zDir;
        prevZ = &lastZ;
    }

    if (trajectory.empty()) {
        return Standard_False;
    }

    if (retract > Precision::Confusion()) {
        const DiscretePoint& seamFirst = trajectory.front();
        const DiscretePoint& seamLast = trajectory.back();

        DiscretePoint startArc = seamFirst;
        startArc.position =
            seamFirst.position.Translated(gp_Vec(seamFirst.zDir) * (-retract));

        DiscretePoint endArc = seamLast;
        endArc.position =
            seamLast.position.Translated(gp_Vec(seamLast.zDir) * (-retract));

        trajectory.insert(trajectory.begin(), startArc);
        trajectory.push_back(endArc);
    }

    return Standard_True;
}

// ---------------------------------------------------------------------------
// 原 WeldSeamTransition.cpp（排序、倒角圆弧、切向插值、Wire 离散）。
// ---------------------------------------------------------------------------

constexpr double kVerticalAngleDeg = 10.0;

void WeldingPathBuilder::GetCirclePath(const std::vector<ProcessedEdge>& edges, TopoDS_Wire& wires)
{
    if (edges.size() < 2) {
        return;
    }

    auto getPt = [](const TopoDS_Edge& edge, gp_Pnt& pntFirst, gp_Pnt& pntLast) -> bool {
        if (edge.IsNull()) {
            return false;
        }
        pntFirst = BRep_Tool::Pnt(TopExp::FirstVertex(edge));
        pntLast = BRep_Tool::Pnt(TopExp::LastVertex(edge));
        return true;
        };

    auto appendSegment = [](const gp_Pnt& a, const gp_Pnt& b, std::vector<TopoDS_Edge>& out) {
        if (a.Distance(b) <= Precision::Confusion()) {
            return;
        }
        BRepBuilderAPI_MakeEdge maker(a, b);
        if (maker.IsDone()) {
            out.push_back(maker.Edge());
        }
        };

    std::vector<TopoDS_Edge> circleEdge;
    for (size_t i = 0; i + 1 < edges.size(); ++i) {
        const ProcessedEdge& pe1 = edges[i];
        const ProcessedEdge& pe2 = edges[i + 1];
        TopoDS_Edge transitionEdge;
        bool ok = false;
        const double distance = pe1.EndPoint.Distance(pe2.StartPoint);
        if (distance <= m_tol) {
            ok = BuildFilletArc(pe1, pe2, transitionEdge);
        }
        else if (distance < 30.0) {
            ok = BuildInterpolationCurve(pe1, pe2, transitionEdge);
        }
        if (ok && !transitionEdge.IsNull()) {
            circleEdge.push_back(transitionEdge);
        }
    }

    if (circleEdge.empty()) {
        return;
    }

    std::vector<TopoDS_Edge> tempEdge;
    gp_Pnt arcStart, arcEnd;
    if (!getPt(circleEdge.front(), arcStart, arcEnd)) {
        return;
    }

    // 第一条焊缝起点接到第一段过渡线
    appendSegment(edges.front().StartPoint, arcStart, tempEdge);

    gp_Pnt prevEnd = arcEnd;
    for (size_t i = 0; i < circleEdge.size(); ++i) {
        gp_Pnt curStart, curEnd;
        if (!getPt(circleEdge[i], curStart, curEnd)) {
            continue;
        }
        if (i > 0) {
            appendSegment(prevEnd, curStart, tempEdge);
        }
        tempEdge.push_back(circleEdge[i]);
        prevEnd = curEnd;
    }

    // 最后一段过渡线再接到最后一条焊缝的终点
    appendSegment(prevEnd, edges.back().EndPoint, tempEdge);

    wires = BuildWiresFromUnorderedEdges(tempEdge, 0.1);
}

bool WeldingPathBuilder::GetTangentAt(const ProcessedEdge& pe, gp_Vec& tangent)
{
    gp_Vec v(pe.StartPoint, pe.EndPoint);
    if (v.Magnitude() < Precision::Confusion()) return false;
    tangent = v.Normalized();
    return true;
}

bool WeldingPathBuilder::BuildInterpolationCurve(const ProcessedEdge& pe1,
    const ProcessedEdge& pe2,
    TopoDS_Edge& outEdge)
{
    const gp_Pnt& p1 = pe1.EndPoint;
    const gp_Pnt& p2 = pe2.StartPoint;

    const Standard_Real chord = p1.Distance(p2);
    // 相邻点距小于公差时，GeomAPI_Interpolate 构造函数直接抛 Standard_ConstructionError。
    // 焊缝首尾已经重合，不需要再插一条过渡曲线。
    if (!(chord > Precision::Confusion())) {
        return false;
    }

    gp_Vec t1;
    gp_Vec t2;
    if (!GetTangentAt(pe1, t1) || !GetTangentAt(pe2, t2)) {
        return false;
    }
    if (t1.Magnitude() <= Precision::Confusion() || t2.Magnitude() <= Precision::Confusion()) {
        return false;
    }
    t1.Normalize();
    t2.Normalize();

    Standard_Real cosA = t1.Dot(t2);
    cosA = std::max(-1.0, std::min(1.0, cosA));
    const Standard_Real theta = std::acos(cosA);

    Standard_Real scale = 0.0;
    if (theta < 1.0e-6) {
        scale = chord / 3.0;
    }
    else {
        const Standard_Real sinHalf = std::sin(theta * 0.5);
        if (!(std::abs(sinHalf) > Precision::Confusion())) {
            return false;
        }
        const Standard_Real radius = chord / (2.0 * sinHalf);
        scale = 0.25 * std::tan(theta * 0.25) * radius;
    }

    const gp_Vec startTangent = t1 * scale;
    const gp_Vec endTangent = t2 * scale;
    // Load() 在切向量长度小于等于公差时同样抛 Standard_ConstructionError。
    if (startTangent.Magnitude() <= Precision::Confusion()
        || endTangent.Magnitude() <= Precision::Confusion()) {
        return false;
    }

    Handle(TColgp_HArray1OfPnt) points = new TColgp_HArray1OfPnt(1, 2);
    points->SetValue(1, p1);
    points->SetValue(2, p2);

    try {
        // 公差必须小于弦长，否则构造函数认为两点重合。
        const Standard_Real fitTol = std::min(Precision::Confusion(), chord * 0.1);
        GeomAPI_Interpolate interp(points, Standard_False, fitTol);
        interp.Load(startTangent, endTangent, Standard_False);
        interp.Perform();
        if (!interp.IsDone()) {
            return false;
        }

        Handle(Geom_BSplineCurve) bspline = interp.Curve();
        if (bspline.IsNull()) {
            return false;
        }
        BRepBuilderAPI_MakeEdge edgeMaker(bspline);
        if (!edgeMaker.IsDone()) {
            return false;
        }
        outEdge = edgeMaker.Edge();
        return true;
    }
    catch (const Standard_Failure&) {
        return false;
    }
}

TopoDS_Wire WeldingPathBuilder::BuildWiresFromUnorderedEdges(
    const std::vector<TopoDS_Edge>& edges, double tolerance)
{
    Handle(TopTools_HSequenceOfShape) edgeSeq = new TopTools_HSequenceOfShape();
    for (const TopoDS_Edge& edge : edges) {
        if (!edge.IsNull() && !BRep_Tool::Degenerated(edge)) {
            edgeSeq->Append(edge);
        }
    }

    TopoDS_Wire result;
    if (edgeSeq->IsEmpty()) {
        return result;
    }

    Handle(TopTools_HSequenceOfShape) wireSeq = new TopTools_HSequenceOfShape();
    // shared = false：端点距离在 tolerance 以内就接到同一条 Wire 上
    ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edgeSeq, tolerance, Standard_False, wireSeq);

    int bestCount = -1;
    for (int i = 1; i <= wireSeq->Length(); ++i) {
        const TopoDS_Wire wire = TopoDS::Wire(wireSeq->Value(i));
        int count = 0;
        for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next()) {
            ++count;
        }
        if (count > bestCount) {
            bestCount = count;
            result = wire;
        }
    }
    return result;
}

bool WeldingPathBuilder::BuildFilletArc(const ProcessedEdge& pe1,
    const ProcessedEdge& pe2,
    TopoDS_Edge& filletEdge)
{
    // 拐角 = 上一条边的终点 = 下一条边的起点（首尾相连）
    const gp_Pnt corner = pe1.EndPoint;

    // ---------- 工具：从 corner 出发、沿 pe 指向远端的单位切向量 ----------
    auto TangentFromCorner = [&](const ProcessedEdge& pe,
        const gp_Pnt& cornerPnt,
        gp_Vec& outTan) -> bool
        {
            // 1) 判断 corner 是 pe 的 StartPoint 还是 EndPoint
            const double dStart = cornerPnt.Distance(pe.StartPoint);
            const double dEnd = cornerPnt.Distance(pe.EndPoint);
            const bool cornerAtStart = (dStart <= dEnd);
            const gp_Pnt& farPnt = cornerAtStart ? pe.EndPoint : pe.StartPoint;

            // 2) 取出底层几何曲线
            Standard_Real first = 0.0, last = 0.0;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(pe.Edge, first, last);

            // 曲线为空（理论上 Edge 有效时不应发生）→ 退化为弦向量
            if (curve.IsNull()) {
                gp_Vec v(cornerPnt, farPnt);
                if (v.Magnitude() < Precision::Confusion()) return false;
                outTan = v.Normalized();
                return true;
            }

            // 3) 把 corner 投影到曲线，拿到对应参数
            Standard_Real param = cornerAtStart ? first : last;
            GeomAPI_ProjectPointOnCurve proj(cornerPnt, curve, first, last);
            if (proj.NbPoints() > 0) {
                param = proj.LowerDistanceParameter();
            }

            // 4) 在该参数处求一阶导（沿"参数增加"方向）
            gp_Pnt p;
            gp_Vec d1Vec;
            curve->D1(param, p, d1Vec);
            if (d1Vec.Magnitude() < Precision::Confusion()) {
                // 退化：用弦向量兜底
                gp_Vec v(cornerPnt, farPnt);
                if (v.Magnitude() < Precision::Confusion()) return false;
                outTan = v.Normalized();
                return true;
            }

            // 5) 把"沿参数增加方向"的切向量转换成"从 corner 指向 farPnt 方向"
            //    判定方式：比较曲线 first/last 端点与 pe.StartPoint 的实际距离，
            //    确定 ProcessedEdge.StartPoint 对应曲线参数小端 还是 大端。
            const gp_Pnt pAtFirst = curve->Value(first);
            const gp_Pnt pAtLast = curve->Value(last);
            const bool peStartAtFirst =
                pe.StartPoint.Distance(pAtFirst) <= pe.StartPoint.Distance(pAtLast);

            // 几何曲线在该端点处"参数前进方向"是否就是"从 corner 朝外"的方向？
            //   情况 A：corner 在 Start，且 Start 对应曲线 first
            //          → 参数前进 = StartPoint→EndPoint = 朝外（不反向）
            //   情况 B：corner 在 Start，且 Start 对应曲线 last
            //          → 参数前进 = EndPoint→StartPoint = 朝里（需要反向）
            //   情况 C：corner 在 End，且 End 对应曲线 last（Start 对应 first）
            //          → 参数前进 = StartPoint→EndPoint = 朝里（需要反向）
            //   情况 D：corner 在 End，且 End 对应曲线 first（Start 对应 last）
            //          → 参数前进 = EndPoint→StartPoint = 朝外（不反向）
            bool needReverse = false;
            if (cornerAtStart) {
                needReverse = !peStartAtFirst;       // A: false / B: true
            }
            else {
                needReverse = peStartAtFirst;        // C: true  / D: false
            }
            if (needReverse) d1Vec.Reverse();

            // 6) 兜底：用弦向量校核方向（防御曲线投影到了反向局部分支等极端情况）
            gp_Vec chord(cornerPnt, farPnt);
            if (chord.Magnitude() > Precision::Confusion()) {
                if (d1Vec.Dot(chord) < 0.0) {
                    d1Vec.Reverse();
                }
            }

            outTan = d1Vec.Normalized();
            return true;
        };

    // ---------- 1. 取出两条边在 corner 处的"朝外"切向量 ----------
    gp_Vec d1, d2;
    if (!TangentFromCorner(pe1, corner, d1)) return false;
    if (!TangentFromCorner(pe2, corner, d2)) return false;

    // ---------- 2. 计算夹角，剔除退化情况 ----------
    const Standard_Real ang = d1.Angle(d2);   // [0, π]
    if (ang < Precision::Angular() ||
        std::abs(ang - M_PI) < Precision::Angular()) {
        // 共线（同向 0 或反向 π）—— 无法构造倒角圆弧
        return false;
    }

    const Standard_Real halfAngle = ang * 0.5;
    const Standard_Real tanHalf = std::tan(halfAngle);
    const Standard_Real sinHalf = std::sin(halfAngle);
    if (std::abs(tanHalf) < Precision::Confusion() ||
        std::abs(sinHalf) < Precision::Confusion()) {
        return false;
    }

    // ---------- 3. 关键长度 ----------
    const Standard_Real tangentDist = m_standoff / tanHalf;   // 切点离 corner 的距离
    const Standard_Real centerDist = m_standoff / sinHalf;   // 圆心离 corner 的距离

    // 切点不能超出任一条边的总长，否则倒角放不下
    if (tangentDist > pe1.Length || tangentDist > pe2.Length) {
        return false;
    }

    // ---------- 4. 角平分线（从 corner 指向圆心一侧） ----------
    gp_Vec bisector = d1 + d2;
    if (bisector.Magnitude() < Precision::Confusion()) {
        // d1, d2 反向（夹角 ≈ π），上面已经排除，这里只是双保险
        return false;
    }
    bisector.Normalize();

    // ---------- 5. 切点 / 圆心 / 圆弧中点 ----------
    const gp_Pnt tp1 = corner.Translated(d1 * tangentDist);
    const gp_Pnt tp2 = corner.Translated(d2 * tangentDist);
    const gp_Pnt center = corner.Translated(bisector * centerDist);
    const gp_Pnt midArc = center.Translated(-bisector * m_standoff);

    // ---------- 6. 三点造圆弧 ----------
    GC_MakeArcOfCircle arcMaker(tp1, midArc, tp2);
    if (!arcMaker.IsDone()) return false;

    Handle(Geom_TrimmedCurve) arcCurve = arcMaker.Value();
    BRepBuilderAPI_MakeEdge edgeMaker(arcCurve);
    if (!edgeMaker.IsDone()) return false;

    filletEdge = edgeMaker.Edge();
    return true;
}

bool WeldingPathBuilder::SortEdges(const std::vector<TopoDS_Edge>& weldEdges,
    std::vector<ProcessedEdge>& sortedEdges,
    std::vector<EdgeCluster>& sortedClusters)
{
    sortedEdges.clear();
    sortedClusters.clear();
    if (weldEdges.empty()) {
        return false;
    }

    // ---------- 常量 ----------
    constexpr double kVerticalAngleDeg = 10.0;     // 与 Z 轴夹角小于该值视为“垂直边”
    constexpr double kCoincidentTol = 0.1;      // 排序阶段：端点共点判定阈值
    constexpr double kClusterMergeTol = 30.0;     // 聚类阶段：簇内相邻非垂直边的最大端点间距
    constexpr double kVecEps = 1.0e-9;   // 零向量阈值

    // ---------- 工具 lambda ----------
    auto GetEdgePoints = [](const TopoDS_Edge& edge, gp_Pnt& start, gp_Pnt& end) {
        if (edge.IsNull()) return;
        TopoDS_Vertex vFirst = TopExp::FirstVertex(edge);
        TopoDS_Vertex vLast = TopExp::LastVertex(edge);
        start = BRep_Tool::Pnt(vFirst);
        end = BRep_Tool::Pnt(vLast);
        };

    auto GetEdgeProperties = [](const TopoDS_Edge& edge, gp_Pnt& center, double& length) {
        GProp_GProps props;
        BRepGProp::LinearProperties(edge, props);
        center = props.CentreOfMass();
        length = props.Mass();
        };

    auto IsVerticalEdge = [&](const gp_Pnt& p1, const gp_Pnt& p2) -> bool {
        gp_Vec v(p1, p2);
        if (v.Magnitude() <= kVecEps) {
            return false;
        }
        gp_Dir d(v);
        double angRad = d.Angle(gp_Dir(0.0, 0.0, 1.0));
        double angDeg = angRad * 180.0 / M_PI;
        if (angDeg > 90.0) angDeg = 180.0 - angDeg;   // 折叠到 [0,90]，方向向下也算
        return angDeg < kVerticalAngleDeg;
        };

    // 按索引判断是否垂直边（供阶段 A “水平优先”排序用）
    auto IsVerticalEdgeIdx = [&](int idx) -> bool {
        gp_Pnt p1, p2;
        GetEdgePoints(weldEdges[idx], p1, p2);
        return IsVerticalEdge(p1, p2);
        };

    auto OrientEdgeEndpoints = [&](const gp_Pnt& p1, const gp_Pnt& p2,
        const gp_Pnt& refPrevEnd,
        bool useRef,
        gp_Pnt& outStart, gp_Pnt& outEnd) {
            if (IsVerticalEdge(p1, p2)) {
                // 【改动】垂直焊缝：从大到小（高 Z 起点，低 Z 终点）
                if (p1.Z() >= p2.Z()) { outStart = p1; outEnd = p2; }
                else { outStart = p2; outEnd = p1; }
                return;
            }
            if (useRef) {
                double d1 = refPrevEnd.Distance(p1);
                double d2 = refPrevEnd.Distance(p2);
                if (d1 <= d2) { outStart = p1; outEnd = p2; }
                else { outStart = p2; outEnd = p1; }
            }
            else {
                if (p1.Y() <= p2.Y()) { outStart = p1; outEnd = p2; }
                else { outStart = p2; outEnd = p1; }
            }
        };

    auto BuildProcessedEdge = [&](const TopoDS_Edge& edge,
        const gp_Pnt& currentEndPnt,
        bool useRef,
        ProcessedEdge& pe) {
            if (edge.IsNull()) return;
            gp_Pnt p1, p2;
            GetEdgePoints(edge, p1, p2);
            OrientEdgeEndpoints(p1, p2, currentEndPnt, useRef, pe.StartPoint, pe.EndPoint);
            pe.Edge = edge;
            GetEdgeProperties(pe.Edge, pe.CenterPoint, pe.Length);
        };

    // ============================================================
    //  阶段 A：全局排序（产出 sortedEdges）
    //  起点：Y 最小的边；串接：贪心最近端点；路口：水平优先
    // ============================================================

    int    currentIdx = -1;
    double minY = std::numeric_limits<double>::max();
    for (size_t i = 0; i < weldEdges.size(); ++i) {
        gp_Pnt p1, p2;
        GetEdgePoints(weldEdges[i], p1, p2);
        double y = std::min(p1.Y(), p2.Y());
        // 如需“起点也强制水平”，可在此处对 y 近似相等时优先选 !IsVerticalEdgeIdx(i)
        if (y < minY) {
            minY = y;
            currentIdx = static_cast<int>(i);
        }
    }
    if (currentIdx < 0) {
        return false;
    }

    std::vector<bool> usedFlags(weldEdges.size(), false);

    ProcessedEdge firstPE;
    BuildProcessedEdge(weldEdges[currentIdx], gp_Pnt(0, 0, 0), /*useRef=*/false, firstPE);
    usedFlags[currentIdx] = true;
    sortedEdges.push_back(firstPE);

    gp_Pnt currentEndPnt = firstPE.EndPoint;

    while (true) {
        std::vector<std::pair<int, double>> adjacent;
        int    nearestIdx = -1;
        double nearestGap = std::numeric_limits<double>::max();

        for (size_t i = 0; i < weldEdges.size(); ++i) {
            if (usedFlags[i]) continue;

            gp_Pnt p1, p2;
            GetEdgePoints(weldEdges[i], p1, p2);
            double d1 = currentEndPnt.Distance(p1);
            double d2 = currentEndPnt.Distance(p2);
            double dMin = std::min(d1, d2);

            if (dMin < kCoincidentTol) {
                adjacent.emplace_back(static_cast<int>(i), dMin);
            }
            if (dMin < nearestGap) {
                nearestGap = dMin;
                nearestIdx = static_cast<int>(i);
            }
        }

        if (adjacent.empty()) {
            if (nearestIdx < 0) break;
            adjacent.emplace_back(nearestIdx, nearestGap);
        }

        int chosenIdx = -1;
        if (adjacent.size() == 1) {
            chosenIdx = adjacent.front().first;
        }
        else {
            std::sort(adjacent.begin(), adjacent.end(),
                [&](const std::pair<int, double>& a,
                    const std::pair<int, double>& b) {
                        // 【改动】水平优先：非垂直边排在前面
                        bool va = IsVerticalEdgeIdx(a.first);
                        bool vb = IsVerticalEdgeIdx(b.first);
                        if (va != vb) return !va;

                        // 同类型：低 Z 优先，再按端点间距
                        gp_Pnt cA, cB;
                        double lA, lB;
                        GetEdgeProperties(weldEdges[a.first], cA, lA);
                        GetEdgeProperties(weldEdges[b.first], cB, lB);
                        if (std::abs(cA.Z() - cB.Z()) > 1e-6) {
                            return cA.Z() < cB.Z();
                        }
                        return a.second < b.second;
                });
            chosenIdx = adjacent.front().first;
        }

        ProcessedEdge nextPE;
        BuildProcessedEdge(weldEdges[chosenIdx], currentEndPnt, /*useRef=*/true, nextPE);
        sortedEdges.push_back(nextPE);
        usedFlags[chosenIdx] = true;

        currentEndPnt = nextPE.EndPoint;

        if (std::find(usedFlags.begin(), usedFlags.end(), false) == usedFlags.end()) {
            break;
        }
    }

    if (sortedEdges.empty()) {
        return false;
    }

    // ============================================================
    //  阶段 B：聚类
    //   1) 垂直边每条单独成簇，并强制切断当前非垂直簇；
    //   2) 非垂直边按“前一条尾点与当前条头点距离 ≤ kClusterMergeTol”聚簇。
    // ============================================================

    auto WithinClusterTol = [&](const gp_Pnt& a, const gp_Pnt& b) -> bool {
        return a.Distance(b) <= kClusterMergeTol;
        };

    std::list<ProcessedEdge> pendingHorz;

    auto FlushPendingHorz = [&]() {
        if (pendingHorz.empty()) return;
        EdgeCluster c;
        c.isVertical = false;
        c.edges.assign(pendingHorz.begin(), pendingHorz.end());
        sortedClusters.push_back(std::move(c));
        pendingHorz.clear();
        };

    for (const ProcessedEdge& pe : sortedEdges) {
        const bool vert = IsVerticalEdge(pe.StartPoint, pe.EndPoint);

        if (vert) {
            FlushPendingHorz();                 // 垂直边强制切断非垂直簇

            EdgeCluster c;
            c.isVertical = true;
            c.edges.push_back(pe);
            sortedClusters.push_back(std::move(c));
            continue;
        }

        if (pendingHorz.empty()) {
            pendingHorz.push_back(pe);
        }
        else {
            const ProcessedEdge& tail = pendingHorz.back();
            if (WithinClusterTol(tail.EndPoint, pe.StartPoint)) {
                pendingHorz.push_back(pe);
            }
            else {
                FlushPendingHorz();
                pendingHorz.push_back(pe);
            }
        }
    }

    FlushPendingHorz();

    return true;
}

std::vector<DiscretePoint> WeldingPathBuilder::DiscretizeWireWithCustomFrame(
    const TopoDS_Wire& wire,
    const TopoDS_Shape& parentShape,
    double stepSize,
    double tolerance,
    double maxAngleStep)
{
    std::vector<DiscretePoint> result;

    const double kEps = std::max(tolerance, 1.0e-9);
    const double kYOrderTol = 1.0e-3;
    if (stepSize <= kEps) return result;
    if (maxAngleStep <= 0.0) maxAngleStep = 5.0 * M_PI / 180.0;

    TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
    if (!parentShape.IsNull()) {
        TopExp::MapShapesAndAncestors(parentShape, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);
    }

    auto AppendFacesOf = [&](const TopoDS_Edge& key, std::vector<TopoDS_Face>& outFaces) {
        if (!edgeFaceMap.Contains(key)) return;
        const TopTools_ListOfShape& faces = edgeFaceMap.FindFromKey(key);
        for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next()) {
            outFaces.push_back(TopoDS::Face(it.Value()));
        }
        };

    // 过渡线往往是新建边，先按同一条边匹配，再按重合且平行的原焊缝边匹配。
    auto GetAdjacentFaces = [&](const TopoDS_Edge& edge, std::vector<TopoDS_Face>& outFaces) {
        outFaces.clear();
        if (edgeFaceMap.IsEmpty()) return;

        if (edgeFaceMap.Contains(edge)) {
            AppendFacesOf(edge, outFaces);
            return;
        }
        for (int i = 1; i <= edgeFaceMap.Extent(); ++i) {
            if (edgeFaceMap.FindKey(i).IsPartner(edge)) {
                AppendFacesOf(TopoDS::Edge(edgeFaceMap.FindKey(i)), outFaces);
                return;
            }
        }

        BRepAdaptor_Curve probe(edge);
        const gp_Pnt mid = probe.Value(0.5 * (probe.FirstParameter() + probe.LastParameter()));
        gp_Vec dir(probe.Value(probe.FirstParameter()), probe.Value(probe.LastParameter()));
        if (dir.Magnitude() <= kEps) return;
        dir.Normalize();

        double bestDist = 1.0;
        TopoDS_Edge bestEdge;
        bool found = false;
        for (int i = 1; i <= edgeFaceMap.Extent(); ++i) {
            const TopoDS_Edge shapeEdge = TopoDS::Edge(edgeFaceMap.FindKey(i));
            if (BRep_Tool::Degenerated(shapeEdge)) continue;

            Standard_Real first = 0.0;
            Standard_Real last = 0.0;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(shapeEdge, first, last);
            if (curve.IsNull()) continue;

            GeomAPI_ProjectPointOnCurve proj(mid, curve, first, last);
            if (proj.NbPoints() < 1) continue;
            const double dist = proj.LowerDistance();
            if (dist >= bestDist) continue;

            gp_Pnt projected;
            gp_Vec edgeTangent;
            curve->D1(proj.LowerDistanceParameter(), projected, edgeTangent);
            if (edgeTangent.Magnitude() <= kEps) continue;
            edgeTangent.Normalize();
            if (std::abs(edgeTangent.Dot(dir)) < 0.99) continue;

            bestDist = dist;
            bestEdge = shapeEdge;
            found = true;
        }
        if (found) {
            AppendFacesOf(bestEdge, outFaces);
        }
        };

    auto EstimateMaxCurvature = [&](BRepAdaptor_Curve& adaptor,
        double pStart, double pEnd, int samples) -> double {
            if (samples < 3) samples = 3;
            double kappaMax = 0.0;
            for (int i = 0; i <= samples; ++i) {
                const double u = pStart + (pEnd - pStart) * static_cast<double>(i) / static_cast<double>(samples);
                gp_Pnt point;
                gp_Vec v1, v2;
                adaptor.D2(u, point, v1, v2);
                const double v1mag = v1.Magnitude();
                if (v1mag <= kEps) continue;
                const double kappa = v1.Crossed(v2).Magnitude() / (v1mag * v1mag * v1mag);
                if (kappa > kappaMax) kappaMax = kappa;
            }
            return kappaMax;
        };

    for (TopExp_Explorer edgeExplorer(wire, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeExplorer.Current());
        BRepAdaptor_Curve curveAdaptor(edge);

        const double firstParam = curveAdaptor.FirstParameter();
        const double lastParam = curveAdaptor.LastParameter();
        if (std::abs(lastParam - firstParam) <= kEps) continue;

        const gp_Pnt paramStart = curveAdaptor.Value(firstParam);
        const gp_Pnt paramEnd = curveAdaptor.Value(lastParam);
        // 参数从 Y 大走向 Y 小时倒序采样，使点列本身从小 Y 走到大 Y。
        const bool reverseParam = paramEnd.Y() + kYOrderTol < paramStart.Y();

        std::vector<TopoDS_Face> adjFaces;
        GetAdjacentFaces(edge, adjFaces);

        const double arcLen = GCPnts_AbscissaPoint::Length(curveAdaptor, kEps);
        if (arcLen <= kEps) continue;

        const int segBase = std::max(1, static_cast<int>(std::ceil(arcLen / stepSize)));
        const double kappaMax = EstimateMaxCurvature(curveAdaptor, firstParam, lastParam, 8);
        int segCurv = 1;
        if (kappaMax > 1.0e-9) {
            segCurv = std::max(1, static_cast<int>(std::ceil(arcLen * kappaMax / maxAngleStep)));
        }
        const int numSegments = std::max(segBase, segCurv);

        std::vector<double> sampleParams;
        sampleParams.reserve(static_cast<std::size_t>(numSegments) + 1);
        GCPnts_UniformAbscissa abscissa(curveAdaptor, numSegments + 1, firstParam, lastParam, kEps);
        if (abscissa.IsDone() && abscissa.NbPoints() >= 2) {
            for (int i = 1; i <= abscissa.NbPoints(); ++i) {
                sampleParams.push_back(abscissa.Parameter(i));
            }
        }
        else {
            const double paramLength = lastParam - firstParam;
            for (int i = 0; i <= numSegments; ++i) {
                sampleParams.push_back(firstParam + paramLength * static_cast<double>(i) / static_cast<double>(numSegments));
            }
        }
        if (reverseParam) {
            std::reverse(sampleParams.begin(), sampleParams.end());
        }

        for (double param : sampleParams) {
            gp_Pnt point;
            gp_Vec tangent;
            curveAdaptor.D1(param, point, tangent);
            if (tangent.Magnitude() <= kEps) {
                const double h = std::max(kEps * 100.0, std::abs(lastParam - firstParam) * 1.0e-4);
                const gp_Pnt pa = curveAdaptor.Value(std::max(firstParam, param - h));
                const gp_Pnt pb = curveAdaptor.Value(std::min(lastParam, param + h));
                tangent = gp_Vec(pa, pb);
                if (tangent.Magnitude() <= kEps) continue;
            }

            gp_Vec tanUnit = tangent.Normalized();
            if (reverseParam) {
                tanUnit.Reverse();
            }

            // 有 Y 向行程时，行走方向指向世界 Y 增大的一侧。
            const double ySpan = paramEnd.Y() - paramStart.Y();
            if (std::abs(ySpan) > kYOrderTol && tanUnit.Y() * ySpan < 0.0) {
                tanUnit.Reverse();
            }

            const gp_Dir yDir(tanUnit);
            const gp_Vec yV(yDir);

            // Z：两侧面外法向的夹角平分线，取向里（指向实体）的一侧，并垂直于行走方向。
            gp_Vec zCand;
            bool hasBisector = false;
            if (adjFaces.size() >= 2) {
                gp_Vec n1, n2;
                if (ComputeFaceNormalAt(adjFaces[0], point, kEps, n1) &&
                    ComputeFaceNormalAt(adjFaces[1], point, kEps, n2)) {
                    gp_Vec bisIn = -(n1 + n2);
                    bisIn -= yV * (bisIn * yV);
                    if (bisIn.Magnitude() > kEps) {
                        zCand = bisIn.Normalized();
                        hasBisector = true;
                    }
                }
            }
            if (!hasBisector) {
                zCand = gp_Vec(0.0, 0.0, 1.0);
                zCand -= yV * (zCand * yV);
                if (zCand.Magnitude() <= kEps) {
                    zCand = gp_Vec(1.0, 0.0, 0.0) ^ yV;
                }
                if (zCand.Magnitude() <= kEps) continue;
                zCand.Normalize();
            }

            // 右手定则：X = Y × Z，再把 Z 正交化为 X × Y。
            gp_Vec xVec = yV ^ zCand;
            if (xVec.Magnitude() <= kEps) continue;
            const gp_Dir xDir(xVec);
            gp_Vec zOrtho = gp_Vec(xDir) ^ yV;
            if (zOrtho.Magnitude() <= kEps) continue;

            DiscretePoint dp;
            dp.position = point;
            dp.xDir = xDir;
            dp.yDir = yDir;
            dp.zDir = gp_Dir(zOrtho);
            result.push_back(dp);
        }
    }

    if (result.empty()) return result;

    std::stable_sort(result.begin(), result.end(),
        [](const DiscretePoint& a, const DiscretePoint& b) {
            return a.position.Y() < b.position.Y();
        });

    const double backDistance = 50.0;
    DiscretePoint newStartPoint = result.front();
    newStartPoint.position.Translate(gp_Vec(newStartPoint.zDir) * (-backDistance));
    DiscretePoint newEndPoint = result.back();
    newEndPoint.position.Translate(gp_Vec(newEndPoint.zDir) * (-backDistance));
    result.insert(result.begin(), newStartPoint);
    result.push_back(newEndPoint);

    return result;
}

// 判断一条边是否近似垂直（用其两端弦向量 vs 世界 Z 轴）
bool WeldingPathBuilder::IsVerticalByEndpoints(const TopoDS_Edge& edge, double vecEps)
{
    BRepAdaptor_Curve adaptor(edge);
    const gp_Pnt p0 = adaptor.Value(adaptor.FirstParameter());
    const gp_Pnt p1 = adaptor.Value(adaptor.LastParameter());
    gp_Vec v(p0, p1);
    if (v.Magnitude() <= vecEps) return false;
    gp_Dir d(v);
    double angRad = d.Angle(gp_Dir(0.0, 0.0, 1.0));
    double angDeg = angRad * 180.0 / M_PI;
    if (angDeg > 90.0) angDeg = 180.0 - angDeg;
    return angDeg < kVerticalAngleDeg;
}

bool WeldingPathBuilder::ComputeFaceNormalAt(const TopoDS_Face& face,
    const gp_Pnt& p,
    double vecEps,
    gp_Vec& outNormal)
{
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    if (surf.IsNull()) return false;

    // 投影点到曲面，得到 (u, v)
    ShapeAnalysis_Surface sas(surf);
    gp_Pnt2d uv = sas.ValueOfUV(p, Precision::Confusion());

    // 求 D1，得到两个偏导，叉乘得几何法向
    gp_Pnt pAt;
    gp_Vec dU, dV;
    surf->D1(uv.X(), uv.Y(), pAt, dU, dV);

    gp_Vec n = dU ^ dV;
    if (n.Magnitude() <= vecEps) return false;
    n.Normalize();

    // face.Orientation()==REVERSED 表示几何法向需要反向，才能指向材料外
    if (face.Orientation() == TopAbs_REVERSED) {
        n.Reverse();
    }
    outNormal = n;
    return true;
}
