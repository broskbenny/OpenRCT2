/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace OpenRCT2::Paint
{
    using FirstPersonTriangleIndices = std::array<uint32_t, 3>;

    struct FirstPersonPlanarMeshReport
    {
        size_t originalTriangles{};
        size_t outputTriangles{};
        size_t replacedPatches{};
        size_t discardedTriangleCount{};
    };

    // Exact triangulation replacement. Vertex positions and all original
    // patch boundary edges remain untouched. Concave polygons are supported;
    // branching boundaries, inner holes and ambiguous ear clipping retain
    // their source triangles instead of risking topology or openings.
    // Does not infer materials, normals across facets or smoother surfaces.
    template<typename TVertex>
    [[nodiscard]] inline std::pair<std::vector<FirstPersonTriangleIndices>, FirstPersonPlanarMeshReport>
        RetessellateFirstPersonConnectedPlanarMesh(
            const std::vector<TVertex>& vertices,
            const std::vector<FirstPersonTriangleIndices>& triangles,
            size_t minimumPatchFaces = 6,
            size_t maximumBoundaryVertices = 256)
    {
        FirstPersonPlanarMeshReport report{};
        report.originalTriangles = triangles.size();
        report.outputTriangles = triangles.size();
        if (triangles.empty() || vertices.empty() || minimumPatchFaces < 2
            || triangles.size() > 100000 || vertices.size() > 100000)
            return {triangles, report};
        struct FacePlane
        {
            std::array<double, 3> normal{};
            double d{};
            bool valid = false;
        };
        const auto get = [&](uint32_t id, size_t axis) -> double {
            const auto& v = vertices[id];
            return axis == 0 ? double(v.x) : axis == 1 ? double(v.y) : double(v.z);
        };
        const auto point = [&](uint32_t id) {
            return std::array<double, 3>{get(id, 0), get(id, 1), get(id, 2)};
        };
        const auto normalOf = [&](const FirstPersonTriangleIndices& tri) {
            const auto a=point(tri[0]), b=point(tri[1]), c=point(tri[2]);
            const std::array<double, 3> u{b[0]-a[0],b[1]-a[1],b[2]-a[2]};
            const std::array<double, 3> v{c[0]-a[0],c[1]-a[1],c[2]-a[2]};
            return std::array<double, 3>{
                u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]};
        };
        std::vector<FacePlane> planes(triangles.size());
        for (size_t i = 0; i < triangles.size(); ++i)
        {
            const auto& tri = triangles[i];
            if (tri[0] >= vertices.size() || tri[1] >= vertices.size()
                || tri[2] >= vertices.size())
                return {triangles, report};
            const auto a = point(tri[0]);
            for (const auto p : {point(tri[1]), point(tri[2])})
                for (const double x : p)
                    if (!std::isfinite(x)) return {triangles, report};
            const auto cross = normalOf(tri);
            const double squared = cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2];
            if (!(squared > 1e-20) || !std::isfinite(squared))
                return {triangles, report};
            const double scale = 1.0 / std::sqrt(squared);
            for (size_t k = 0; k < 3; ++k)
                planes[i].normal[k] = cross[k] * scale;
            planes[i].d = planes[i].normal[0]*a[0]
                + planes[i].normal[1]*a[1]+planes[i].normal[2]*a[2];
            planes[i].valid = true;
        }
        const auto edgeKey = [](uint32_t a, uint32_t b) -> uint64_t {
            if (a > b) std::swap(a,b);
            return (uint64_t(a)<<32)|b;
        };
        std::unordered_map<uint64_t, std::vector<size_t>> adjacency;
        adjacency.reserve(triangles.size()*3);
        for (size_t i = 0; i < triangles.size(); ++i)
            for (size_t k = 0; k < 3; ++k)
                adjacency[edgeKey(triangles[i][k],triangles[i][(k+1)%3])].push_back(i);
        std::vector<size_t> parents(triangles.size());
        for (size_t i = 0; i < parents.size(); ++i) parents[i] = i;
        const auto root = [&](size_t i) -> size_t {
            while (parents[i] != i) {parents[i] = parents[parents[i]];i = parents[i];}
            return i;
        };
        for (const auto& [key, pair] : adjacency)
        {
            if (pair.size() != 2) continue;
            const auto& a = planes[pair[0]], &b = planes[pair[1]];
            const double cosine = a.normal[0]*b.normal[0]
                +a.normal[1]*b.normal[1]+a.normal[2]*b.normal[2];
            if (cosine <= 0.999999 || std::abs(a.d-b.d) >= 1e-5) continue;
            const size_t ra=root(pair[0]), rb=root(pair[1]);
            if (ra!=rb) parents[rb]=ra;
        }
        std::map<size_t, std::vector<size_t>> groups;
        for (size_t i = 0; i < triangles.size(); ++i)
            groups[root(i)].push_back(i);
        std::vector<FirstPersonTriangleIndices> out;
        out.reserve(triangles.size());
        const auto appendOriginal = [&](const std::vector<size_t>& group) {
            for (const auto i : group) out.push_back(triangles[i]);
        };
        for (const auto& [id, group] : groups)
        {
            if (group.size() < minimumPatchFaces) {appendOriginal(group);continue;}
            std::unordered_map<uint64_t, size_t> counts;
            counts.reserve(group.size()*3);
            for (const size_t i : group)
                for (size_t k = 0; k < 3; ++k)
                    ++counts[edgeKey(triangles[i][k],triangles[i][(k+1)%3])];
            std::unordered_map<uint32_t, uint32_t> successor;
            std::unordered_map<uint32_t, size_t> indegree;
            bool bad = false;
            size_t boundaryEdges = 0;
            for (const size_t i : group)
            for (size_t k = 0; k < 3; ++k)
            {
                const uint32_t a=triangles[i][k], b=triangles[i][(k+1)%3];
                const auto count=counts[edgeKey(a,b)];
                if (count>2) {bad=true;break;}
                if (count != 1) continue;
                ++boundaryEdges;
                if (!successor.emplace(a,b).second) bad=true;
                ++indegree[b];
            }
            if (bad || boundaryEdges<3 || boundaryEdges>maximumBoundaryVertices
                || successor.size()!=boundaryEdges)
            {appendOriginal(group);continue;}
            for (const auto& [a,b] : successor)
                if (indegree[a]!=1) bad=true;
            if (bad) {appendOriginal(group);continue;}
            std::vector<uint32_t> loop;
            loop.reserve(boundaryEdges);
            uint32_t here=successor.begin()->first, start=here;
            std::unordered_set<uint32_t> seen;
            do {
                if (!seen.insert(here).second) {bad=true;break;}
                loop.push_back(here);
                here=successor[here];
            } while (here!=start && loop.size()<=boundaryEdges);
            if (bad || here!=start || loop.size()!=boundaryEdges)
            {appendOriginal(group);continue;} // Includes holes and multiple loops.
            const auto normal=planes[group.front()].normal;
            size_t drop=0;
            if (std::abs(normal[1])>std::abs(normal[drop])) drop=1;
            if (std::abs(normal[2])>std::abs(normal[drop])) drop=2;
            const size_t axisU=(drop+1)%3,axisV=(drop+2)%3;
            const auto cross2=[&](uint32_t a,uint32_t b,uint32_t c) {
                const double ux=get(b,axisU)-get(a,axisU);
                const double uy=get(b,axisV)-get(a,axisV);
                const double vx=get(c,axisU)-get(a,axisU);
                const double vy=get(c,axisV)-get(a,axisV);
                return ux*vy-uy*vx;
            };
            double area=0;
            for (size_t i = 0; i < loop.size(); ++i)
                area+=get(loop[i],axisU)*get(loop[(i+1)%loop.size()],axisV)
                    -get(loop[(i+1)%loop.size()],axisU)*get(loop[i],axisV);
            const double sign=area>0?1.0:-1.0;
            if (std::abs(area)<1e-8) {appendOriginal(group);continue;}
            std::vector<uint32_t> polygon=loop;
            std::vector<FirstPersonTriangleIndices> replacement;
            replacement.reserve(loop.size()-2);
            // Keep all boundary vertices: deleting a collinear border vertex
            // would make a T-junction against its neighbouring 3D patch.
            while (polygon.size()>3)
            {
                bool found=false;
                for (size_t k=0;k<polygon.size();++k)
                {
                    const size_t n=polygon.size();
                    const uint32_t a=polygon[(k+n-1)%n],b=polygon[k],c=polygon[(k+1)%n];
                    const double earArea=cross2(a,b,c)*sign;
                    if (earArea<=1e-9)continue;
                    bool occupied=false;
                    for (const auto p : polygon)
                    {
                        if (p==a||p==b||p==c)continue;
                        // A point ON an ear edge must also be preserved.
                        if (cross2(a,b,p)*sign>=-1e-9
                            && cross2(b,c,p)*sign>=-1e-9
                            && cross2(c,a,p)*sign>=-1e-9)
                        {occupied=true;break;}
                    }
                    if (occupied)continue;
                    replacement.push_back({a,b,c});
                    polygon.erase(polygon.begin()+k);
                    found=true;break;
                }
                if (!found){bad=true;break;}
            }
            if (!bad)replacement.push_back({polygon[0],polygon[1],polygon[2]});
            if (bad || replacement.size()>=group.size())
            {appendOriginal(group);continue;}
            // Verify both oriented patch winding, exact border edges and
            // source-area preservation. Matching contour edges alone would
            // not detect a self-overlapping polygon or a folded triangulation.
            const auto area3 = [&](const FirstPersonTriangleIndices& tri) {
                const auto c = normalOf(tri);
                return std::sqrt(c[0]*c[0]+c[1]*c[1]+c[2]*c[2])*0.5;
            };
            double originalArea=0.0, outputArea=0.0;
            for (size_t i : group) originalArea += area3(triangles[i]);
            for (const auto& tri : replacement) outputArea += area3(tri);
            if (!std::isfinite(outputArea)
                || std::abs(originalArea-outputArea)
                    > std::max(1e-6, originalArea*1e-7))
            {appendOriginal(group);continue;}
            std::unordered_map<uint64_t, size_t> newCounts;
            for (auto& tri : replacement)
            {
                const auto cross=normalOf(tri);
                const double orientation=cross[0]*normal[0]
                    +cross[1]*normal[1]+cross[2]*normal[2];
                if (std::abs(orientation)<1e-8){bad=true;break;}
                if (orientation<0)std::swap(tri[1],tri[2]);
                for (size_t k=0;k<3;++k)
                    ++newCounts[edgeKey(tri[k],tri[(k+1)%3])];
            }
            if (!bad)
                for (const auto& [key,count] : counts)
                    if ((count==1)!=(newCounts[key]==1)) {bad=true;break;}
            if (bad){appendOriginal(group);continue;}
            out.insert(out.end(),replacement.begin(),replacement.end());
            report.replacedPatches++;
            report.discardedTriangleCount+=group.size()-replacement.size();
        }
        report.outputTriangles=out.size();
        return {std::move(out),report};
    }
} // namespace OpenRCT2::Paint
