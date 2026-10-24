// This file is included inside namespace OpenRCT2::Paint by FirstPersonVisualHull.h.
// Four native orthographic silhouettes can contain structural strokes whose
// 3D direction is not a voxel-lattice direction. Find long 2D stroke segments,
// intersect their back-projected PLANES in world space, and certify every
// resulting 3D prism against all four native source masks.
//
// No proprietary DAT content, per-object templates, or scene IDs are used.
template<typename ProjectPoint>
[[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
    BuildFirstPersonSourceTriangulatedWireFaces(
        const FirstPersonVisualHull& hull,
        const std::vector<FirstPersonVisualHullView>& views,
        ProjectPoint&& projectPoint,
        size_t originalFaceCount, size_t maximumFaces = 768,
        bool prioritizeNativeCoverage = false,
        bool includeAllSourceViewPairs = false)
{
    // A zero baseline means voxel carving never produced a candidate.
    // Native stroke triangulation is independently evidence-based, so it
    // may still infer a mesh; there is then no voxel face count to beat.
    if (!hull.valid || views.size() != 4
        || (originalFaceCount != 0 && originalFaceCount < 200)
        || maximumFaces < 24 || hull.occupied.empty())
        return {};

    std::array<const FirstPersonVisualHullView*, 4> native{};
    for (const auto& v : views)
    {
        if (v.imageDirection >= 4 || native[v.imageDirection] != nullptr
            || v.observed.empty() || v.observed.size() < 250
            || v.observed.size() > 5000)
            return {};
        const float area = float(v.observed.maxX - v.observed.minX)
            * float(v.observed.maxY - v.observed.minY);
        // A mostly solid wall is not a perforated architectural truss.
        if (area < 400.0f || float(v.observed.size()) / area > 0.60f)
            return {};
        native[v.imageDirection] = &v;
    }

    struct Line
    {
        std::array<float, 2> p{}, q{}, tangent{}, normal{};
        float length{};
    };
    struct Pixel
    {
        int32_t x{}, y{};
        float along{};
    };
    constexpr int kAngles = 36;
    constexpr int kRhoBins = 256;
    constexpr float kPi = 3.14159265358979323846f;
    std::array<std::vector<Line>, 4> lines{};
    for (int d = 0; d < 4; ++d)
    {
        // Bin relative to the native image bounds, not its arbitrary
        // drawing offset. This makes stroke inference invariant under
        // screen-space registration and halves the bounded Hough storage.
        const int32_t centreX = native[d]->observed.minX
            + (native[d]->observed.maxX - native[d]->observed.minX) / 2;
        const int32_t centreY = native[d]->observed.minY
            + (native[d]->observed.maxY - native[d]->observed.minY) / 2;
        std::vector<std::vector<Pixel>> buckets(size_t(kAngles * kRhoBins));
        for (uint64_t id : native[d]->observed.pixels)
        {
            const int32_t x = FirstPersonSilhouettePixelX(id);
            const int32_t y = FirstPersonSilhouettePixelY(id);
            for (int a = 0; a < kAngles; ++a)
            {
                const float angle = float(a) * kPi / float(kAngles);
                const float c = std::cos(angle), s = std::sin(angle);
                const int32_t rho = int32_t(std::lround(
                    float(x - centreX) * c + float(y - centreY) * s));
                if (rho <= -128 || rho >= 128)
                    continue;
                buckets[size_t(a*kRhoBins + rho + 128)]
                    .push_back({ x, y, -float(x)*s + float(y)*c });
            }
        }
        for (size_t bin = 0; bin < buckets.size(); ++bin)
        {
            auto& candidates = buckets[bin];
            if (candidates.size() < 6)
                continue;
            std::sort(candidates.begin(), candidates.end(),
                [](const Pixel& a, const Pixel& b) { return a.along < b.along; });
            size_t first = 0;
            for (size_t k = 1; k <= candidates.size(); ++k)
            {
                if (k < candidates.size()
                    && candidates[k].along - candidates[k - 1].along <= 2.5f)
                    continue;
                if (k - first >= 6)
                {
                    const auto& p = candidates[first];
                    const auto& q = candidates[k - 1];
                    const float dx = float(q.x - p.x);
                    const float dy = float(q.y - p.y);
                    const float length = std::sqrt(dx*dx + dy*dy);
                    if (length >= 6.0f)
                    {
                        Line line{};
                        line.p = { float(p.x), float(p.y) };
                        line.q = { float(q.x), float(q.y) };
                        line.tangent = { dx / length, dy / length };
                        line.normal = { -dy / length, dx / length };
                        line.length = length;
                        lines[d].push_back(line);
                    }
                }
                first = k;
            }
        }
        std::stable_sort(lines[d].begin(), lines[d].end(),
            [](const Line& a, const Line& b) { return a.length > b.length; });
        // Native ornament masks contain thousands of overlapping Hough
        // strokes. Choosing the longest 80 can spend most of the budget on
        // the same thick bar and miss genuine cross-members. First build a
        // bounded, nonduplicate pool, then retain strokes by the NEW native
        // pixels they explain. The subsequent 3D inference and four-view
        // admission remain authoritative.
        constexpr size_t kMaximumLinePool = 600;
        constexpr size_t kMaximumSelectedLines = 80;
        std::vector<Line> distinct;
        for (const auto& line : lines[d])
        {
            const float mx = (line.p[0]+line.q[0])*0.5f;
            const float my = (line.p[1]+line.q[1])*0.5f;
            bool duplicate = false;
            for (const auto& other : distinct)
            {
                const float ox = (other.p[0]+other.q[0])*0.5f;
                const float oy = (other.p[1]+other.q[1])*0.5f;
                const float align = std::abs(
                    line.tangent[0]*other.tangent[0]
                    + line.tangent[1]*other.tangent[1]);
                if (align > 0.985f && std::abs(mx-ox) < 1.5f
                    && std::abs(my-oy) < 1.5f)
                {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate)
                distinct.push_back(line);
            if (distinct.size() >= (prioritizeNativeCoverage
                    ? kMaximumLinePool : kMaximumSelectedLines))
                break;
        }
        // Preserve the previously certified longest-stroke hypothesis. The
        // coverage-based selection is a distinct competing 3D candidate:
        // 2D coverage improvement alone must never replace a valid truss.
        if (!prioritizeNativeCoverage)
        {
            lines[d] = std::move(distinct);
            if (lines[d].size() < 2)
                return {};
            continue;
        }
        // Rasterize each proposed centreline ONCE at source-pixel resolution.
        // A line crossing native transparency is penalized, not silently
        // filled; no indexed artwork pixels are changed by this selection.
        std::vector<std::vector<uint64_t>> footprints(distinct.size());
        for (size_t i = 0; i < distinct.size(); ++i)
        {
            const auto& line = distinct[i];
            int32_t x = int32_t(std::lround(line.p[0]));
            int32_t y = int32_t(std::lround(line.p[1]));
            const int32_t endX = int32_t(std::lround(line.q[0]));
            const int32_t endY = int32_t(std::lround(line.q[1]));
            const int32_t dx = std::abs(endX - x);
            const int32_t dy = -std::abs(endY - y);
            const int32_t sx = x < endX ? 1 : -1;
            const int32_t sy = y < endY ? 1 : -1;
            int32_t error = dx + dy;
            for (;;)
            {
                footprints[i].push_back(FirstPersonSilhouettePixelKey(x, y));
                if (x == endX && y == endY)
                    break;
                const int32_t twiceError = 2 * error;
                if (twiceError >= dy)
                {
                    error += dy;
                    x += sx;
                }
                if (twiceError <= dx)
                {
                    error += dx;
                    y += sy;
                }
            }
        }
        std::vector<uint8_t> selected(distinct.size(), 0);
        std::unordered_set<uint64_t> coveredStrokes;
        std::vector<Line> selectedLines;
        selectedLines.reserve(kMaximumSelectedLines);
        for (size_t count = 0; count < kMaximumSelectedLines; ++count)
        {
            size_t best = distinct.size();
            int32_t bestScore = 0;
            for (size_t i = 0; i < distinct.size(); ++i)
            {
                if (selected[i])
                    continue;
                int32_t observedPixels = 0;
                int32_t unobservedPixels = 0;
                for (uint64_t pixel : footprints[i])
                {
                    if (coveredStrokes.contains(pixel))
                        continue;
                    if (native[d]->observed.pixels.contains(pixel))
                        ++observedPixels;
                    else
                        ++unobservedPixels;
                }
                const int32_t score =
                    5 * observedPixels - 2 * unobservedPixels;
                if (score > bestScore)
                {
                    bestScore = score;
                    best = i;
                }
            }
            if (best == distinct.size())
                break;
            selected[best] = 1;
            selectedLines.push_back(distinct[best]);
            coveredStrokes.insert(
                footprints[best].begin(), footprints[best].end());
        }
        lines[d] = std::move(selectedLines);
        if (lines[d].size() < 2)
            return {};
    }

    const auto add = [](FirstPersonVec3 a, FirstPersonVec3 b) {
        return FirstPersonVec3{ a.x+b.x, a.y+b.y, a.z+b.z };
    };
    const auto sub = [](FirstPersonVec3 a, FirstPersonVec3 b) {
        return FirstPersonVec3{ a.x-b.x, a.y-b.y, a.z-b.z };
    };
    const auto mul = [](FirstPersonVec3 a, float t) {
        return FirstPersonVec3{ a.x*t, a.y*t, a.z*t };
    };
    const auto unit = [&](FirstPersonVec3 a) {
        const float l=std::sqrt(FpDot(a,a));
        return l>1.e-6f ? mul(a,1.0f/l) : FirstPersonVec3{};
    };
    struct Projection
    {
        std::array<float, 2> origin{};
        std::array<FirstPersonVec3, 2> gradients{};
    };
    std::array<Projection, 4> projection{};
    for (uint8_t d=0;d<4;++d)
    {
        auto& p=projection[d];
        p.origin=projectPoint(d,{0,0,0});
        const auto px=projectPoint(d,{32,0,0});
        const auto py=projectPoint(d,{0,32,0});
        const auto pz=projectPoint(d,{0,0,32});
        p.gradients[0]={
            (px[0]-p.origin[0])/32.0f,
            (py[0]-p.origin[0])/32.0f,
            (pz[0]-p.origin[0])/32.0f
        };
        p.gradients[1]={
            (px[1]-p.origin[1])/32.0f,
            (py[1]-p.origin[1])/32.0f,
            (pz[1]-p.origin[1])/32.0f
        };
    }
    const auto plane = [&](const Line& line,int d) {
        return add(mul(projection[d].gradients[0],line.normal[0]),
                   mul(projection[d].gradients[1],line.normal[1]));
    };
    const auto rhs = [&](const Line& line,int d) {
        return line.normal[0]*(line.p[0]-projection[d].origin[0])
            + line.normal[1]*(line.p[1]-projection[d].origin[1]);
    };
    const auto bound = [](float base,float direction,float low,float high,
                          float& begin,float& end) {
        if (std::abs(direction)<1.e-6f)
            return base >= low && base <= high;
        float a=(low-base)/direction, b=(high-base)/direction;
        if (a>b)
            std::swap(a,b);
        begin=std::max(begin,a);
        end=std::min(end,b);
        return begin<end;
    };
    const auto constrainLine = [&](FirstPersonVec3 anchor,FirstPersonVec3 axis,
                                   const Line& line,int d,
                                   float& begin,float& end) {
        const auto pa=projectPoint(uint8_t(d),anchor);
        const auto pb=projectPoint(uint8_t(d),add(anchor,mul(axis,32.0f)));
        const float at=(pa[0]-line.p[0])*line.tangent[0]
            +(pa[1]-line.p[1])*line.tangent[1];
        const float rate=((pb[0]-pa[0])*line.tangent[0]
            +(pb[1]-pa[1])*line.tangent[1])/32.0f;
        return bound(at,rate,-1.0f,line.length+1.0f,begin,end);
    };
    struct Hypothesis
    {
        FirstPersonVec3 p{},q{};
        float score{};
    };
    std::vector<Hypothesis> proposals;
    // Every unordered pair of independent native orthographic directions
    // can constrain a 3D source stroke. The previous four-pair subset
    // omitted (0,3) and (1,2), arbitrarily excluding real diagonal
    // members. Preserve the prior four first for stable tie ordering.
    // Proposal evaluation is still bounded by the existing four-view
    // support checks and the 400-rod projection-candidate cap below.
    constexpr int pairs[6][2]={
        {0,1},{0,2},{1,3},{2,3},{0,3},{1,2}
    };
    const FirstPersonVec3 centre{
        hull.minForward+float(hull.sizeForward)*hull.step*.5f,
        hull.minRight+float(hull.sizeRight)*hull.step*.5f,
        hull.minUp+float(hull.sizeUp)*hull.step*.5f
    };
    // Preserve the historical four-pair winner on the normal path.
    // The two extra stereo constraints are a fallback for candidates that
    // fail independent four-view admission, not a source of regressions.
    for (int pairIndex=0;
         pairIndex<(includeAllSourceViewPairs ? 6 : 4); ++pairIndex)
    {
        const auto& pair=pairs[pairIndex];
        const int d0=pair[0],d1=pair[1];
        for (const auto& l0 : lines[d0])
        for (const auto& l1 : lines[d1])
        {
            const auto n0=plane(l0,d0),n1=plane(l1,d1);
            const auto cross=FpCross(n0,n1);
            const float norm=std::sqrt(FpDot(cross,cross));
            if (norm<0.03f)
                continue;
            const auto axis=mul(cross,1.0f/norm);
            const auto n1x=FpCross(n1,axis);
            const auto nx0=FpCross(axis,n0);
            const float det=FpDot(n0,n1x);
            if (std::abs(det)<1.e-4f)
                continue;
            const auto anchor=mul(
                add(add(mul(n1x,rhs(l0,d0)),mul(nx0,rhs(l1,d1))),
                    mul(cross,FpDot(axis,centre))),1.0f/det);
            float lo=-1000.0f,hi=1000.0f;
            if (!bound(anchor.x,axis.x,hull.minForward,
                    hull.minForward+float(hull.sizeForward)*hull.step,lo,hi)
                || !bound(anchor.y,axis.y,hull.minRight,
                    hull.minRight+float(hull.sizeRight)*hull.step,lo,hi)
                || !bound(anchor.z,axis.z,hull.minUp,
                    hull.minUp+float(hull.sizeUp)*hull.step,lo,hi)
                || !constrainLine(anchor,axis,l0,d0,lo,hi)
                || !constrainLine(anchor,axis,l1,d1,lo,hi)
                || hi-lo<6.0f)
                continue;
            const int count=std::clamp(int(std::ceil((hi-lo)*2.0f)),8,120);
            int exactTotal=0;
            float minimumNear=1.0f;
            for (int d=0;d<4;++d)
            {
                int exact=0,close=0;
                for (int i=0;i<count;++i)
                {
                    const float t=lo+(hi-lo)*(float(i)+.5f)/float(count);
                    const auto uv=projectPoint(uint8_t(d),
                        add(anchor,mul(axis,t)));
                    const int x=int(std::lround(uv[0]));
                    const int y=int(std::lround(uv[1]));
                    if (native[d]->observed.contains(x,y))
                        ++exact;
                    bool supported=false;
                    for (int dy=-1;dy<=1 && !supported;++dy)
                    for (int dx=-1;dx<=1;++dx)
                        if (native[d]->observed.contains(x+dx,y+dy))
                        {
                            supported=true;
                            break;
                        }
                    if (supported)
                        ++close;
                }
                minimumNear=std::min(minimumNear,float(close)/float(count));
                exactTotal+=exact;
            }
            if (minimumNear<0.80f
                || exactTotal*5<count*8)
                continue;
            const float score=4.0f*minimumNear
                +float(exactTotal)/float(count)+.005f*(hi-lo);
            proposals.push_back({
                add(anchor,mul(axis,lo)),add(anchor,mul(axis,hi)),score
            });
        }
    }
    if (proposals.size()<2)
        return {};
    std::stable_sort(proposals.begin(),proposals.end(),
        [](const Hypothesis& a,const Hypothesis& b) {
            return a.score>b.score;
        });
    constexpr size_t kMaximumProjections=400;
    if (proposals.size()>kMaximumProjections)
        proposals.resize(kMaximumProjections);

    const auto makeFaces = [&](FirstPersonVec3 start,FirstPersonVec3 end,
                               float radius=.45f) {
        std::vector<FirstPersonVisualHullFace> out;
        const auto direction=unit(sub(end,start));
        const auto vertical=std::abs(direction.z)<.8f
            ? FirstPersonVec3{0,0,1} : FirstPersonVec3{0,1,0};
        const auto side=unit(FpCross(direction,vertical));
        const auto across=unit(FpCross(direction,side));
        std::array<FirstPersonVec3,4> startRing{},endRing{};
        for (int i=0;i<4;++i)
        {
            const float a=i==0 || i==3 ? radius:-radius;
            const float b=i<2 ? radius:-radius;
            const auto v=add(mul(side,a),mul(across,b));
            startRing[i]=add(start,v);
            endRing[i]=add(end,v);
        }
        const auto quad=[&](std::array<FirstPersonVec3,4> corners,
                            FirstPersonVec3 outward) {
            auto normal=unit(FpCross(sub(corners[1],corners[0]),
                                     sub(corners[2],corners[0])));
            if (FpDot(normal,normal)<.5f)
                return;
            if (FpDot(normal,outward)<0)
            {
                std::swap(corners[1],corners[3]);
                normal=mul(normal,-1);
            }
            FirstPersonVisualHullFace f{};
            f.corners=corners;f.normal=normal;
            f.kind=normal.z<-.7f ? FirstPersonVisualHullFaceKind::bottom
                : normal.z>.7f ? FirstPersonVisualHullFaceKind::top
                : std::abs(normal.x)>std::abs(normal.y)
                    ? (normal.x<0 ? FirstPersonVisualHullFaceKind::minForward
                                   : FirstPersonVisualHullFaceKind::maxForward)
                    : (normal.y<0 ? FirstPersonVisualHullFaceKind::minRight
                                   : FirstPersonVisualHullFaceKind::maxRight);
            out.push_back(f);
        };
        quad({startRing[3],startRing[2],startRing[1],startRing[0]},
            mul(direction,-1));
        quad({endRing[0],endRing[1],endRing[2],endRing[3]},direction);
        for (int i=0;i<4;++i)
        {
            const int j=(i+1)&3;
            quad({startRing[i],startRing[j],endRing[j],endRing[i]},
                add(sub(startRing[i],start),sub(startRing[j],start)));
        }
        return out;
    };
    struct Candidate
    {
        std::vector<FirstPersonVisualHullFace> faces;
        std::array<FirstPersonSilhouette,4> projected{};
        // Preserve the inferred world-space centreline for bounded junction
        // inference after the independently certified member selection.
        FirstPersonVec3 p{}, q{};
    };
    std::vector<Candidate> candidates;
    for (const auto& proposal : proposals)
    {
        auto faces=makeFaces(proposal.p,proposal.q);
        if (faces.size()!=6)
            continue;
        auto triangles=TriangulateFirstPersonReconstructionFaces(faces);
        if (triangles.empty())
            continue;
        Candidate c{};
        c.faces=std::move(faces);
        c.p=proposal.p;
        c.q=proposal.q;
        bool valid=true;
        for (int d=0;d<4;++d)
        {
            auto predicted=RenderFirstPersonReconstructionSilhouette(
                triangles,uint8_t(d),projectPoint);
            if (predicted.empty() || predicted.overflowed)
            {
                valid=false;
                break;
            }
            c.projected[d]=std::move(predicted);
        }
        if (valid)
            candidates.push_back(std::move(c));
    }
    if (candidates.size()<2)
        return {};
    std::array<std::unordered_set<uint64_t>,4> covered{};
    std::vector<uint8_t> chosen(candidates.size(),0);
    const size_t maxBeams=std::min<size_t>(120,maximumFaces/6);
    size_t count=0;
    for (;count<maxBeams;++count)
    {
        int64_t bestScore=0;
        size_t best=candidates.size();
        for (size_t i=0;i<candidates.size();++i)
        {
            if (chosen[i])
                continue;
            int64_t present=0,missing=0;
            for (int d=0;d<4;++d)
            {
                for (uint64_t px : candidates[i].projected[d].pixels)
                {
                    if (covered[d].contains(px))
                        continue;
                    if (native[d]->observed.pixels.contains(px))
                        ++present;
                    else
                        ++missing;
                }
            }
            const int64_t score=2*(present-missing);
            if (score>bestScore)
            {
                bestScore=score;
                best=i;
            }
        }
        if (best==candidates.size())
            break;
        chosen[best]=1;
        for (int d=0;d<4;++d)
        {
            const auto& px=candidates[best].projected[d].pixels;
            covered[d].insert(px.begin(),px.end());
        }
    }
    if (count<8)
        return {};

    // Identical native-view projections do not imply identical geometry.
    // Remove each selected rod that contributes no unique silhouette pixel:
    // such rods can create phantom structure from novel camera directions.
    // Count unsupported projections too, preserving the exact four-view
    // silhouette union (and thus the independent admission certificate).
    std::array<std::unordered_map<uint64_t, uint16_t>, 4> projectedUses{};
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        if (!chosen[i])
            continue;
        for (int d = 0; d < 4; ++d)
        {
            for (uint64_t pixel : candidates[i].projected[d].pixels)
                ++projectedUses[d][pixel];
        }
    }
    // Reverse proposal quality order: prefer retaining the better-supported
    // hypothesis when multiple rods explain precisely the same native pixels.
    for (size_t i = candidates.size(); i-- > 0;)
    {
        if (!chosen[i])
            continue;
        bool hasUniqueProjection = false;
        for (int d = 0; d < 4 && !hasUniqueProjection; ++d)
        {
            for (uint64_t pixel : candidates[i].projected[d].pixels)
            {
                if (projectedUses[d].at(pixel) == 1)
                {
                    hasUniqueProjection = true;
                    break;
                }
            }
        }
        if (hasUniqueProjection)
            continue;
        chosen[i] = 0;
        --count;
        for (int d = 0; d < 4; ++d)
        {
            for (uint64_t pixel : candidates[i].projected[d].pixels)
            {
                auto it = projectedUses[d].find(pixel);
                if (--it->second == 0)
                    projectedUses[d].erase(it);
            }
        }
    }

    std::vector<FirstPersonVisualHullFace> result;
    result.reserve(count*6);
    for (size_t i=0;i<candidates.size();++i)
        if (chosen[i])
            result.insert(result.end(),
                candidates[i].faces.begin(),candidates[i].faces.end());
    if (result.size()>maximumFaces
        || (originalFaceCount != 0
            && result.size()*3>=originalFaceCount*2))
        return {};

    // The selected world-space members are independently inferred, but
    // treating each as a closed prism leaves many near T-junctions floating.
    // Connect nearby rod ENDPOINTS to neighbouring world-space centreline
    // segments without ever moving the authored, selected rod geometry.
    //
    // Project every proposed junction through all four real native source
    // views. Admit only if EVERY pixel is simultaneously covered by the
    // selected rod union and by opaque native artwork. Thus a connector
    // introduces zero new silhouette pixels, cannot worsen the certificate,
    // and cannot justify promoting an otherwise rejected reconstruction.
    // This is a real 3D connection, not a camera-facing sprite or an
    // independently invented source colour.
    const size_t faceLimit = originalFaceCount == 0
        ? maximumFaces
        : std::min(maximumFaces,(originalFaceCount*2-1)/3);
    if (result.size()+6>faceLimit)
        return result;
    struct Junction
    {
        FirstPersonVec3 start{},end{};
        float distanceSquared{};
    };
    std::vector<Junction> junctions;
    constexpr float kJoinDistanceSquared=2.25f;
    constexpr float kMinimumJoinDistanceSquared=.0625f;
    for (size_t i=0;i<candidates.size();++i)
    {
        if (!chosen[i])
            continue;
        for (const FirstPersonVec3 start :
             { candidates[i].p,candidates[i].q })
        {
            float best=kJoinDistanceSquared;
            FirstPersonVec3 target{};
            bool found=false;
            for (size_t j=0;j<candidates.size();++j)
            {
                if (i==j || !chosen[j])
                    continue;
                const auto& a=candidates[j].p;
                const auto span=sub(candidates[j].q,a);
                const float lengthSquared=FpDot(span,span);
                if (lengthSquared<1.0f)
                    continue;
                const float along=std::clamp(
                    FpDot(sub(start,a),span)/lengthSquared,0.0f,1.0f);
                const auto projected=add(a,mul(span,along));
                const auto delta=sub(start,projected);
                const float distanceSquared=FpDot(delta,delta);
                if (distanceSquared<best
                    && distanceSquared>=kMinimumJoinDistanceSquared)
                {
                    best=distanceSquared;
                    target=projected;
                    found=true;
                }
            }
            if (found)
                junctions.push_back({start,target,best});
        }
    }
    std::stable_sort(junctions.begin(),junctions.end(),
        [](const Junction& a,const Junction& b) {
            return a.distanceSquared<b.distanceSquared;
        });
    for (const auto& joint : junctions)
    {
        if (result.size()+6>faceLimit)
            break;
        auto faces=makeFaces(joint.start,joint.end,.35f);
        if (faces.size()!=6)
            continue;
        const auto triangles=TriangulateFirstPersonReconstructionFaces(faces);
        if (triangles.empty())
            continue;
        bool supported=true;
        for (int d=0;d<4 && supported;++d)
        {
            const auto junctionProjection=RenderFirstPersonReconstructionSilhouette(
                triangles,uint8_t(d),projectPoint);
            // All four projected source views must witness the connector;
            // a wholly invisible tube has no trustworthy native material.
            if (junctionProjection.empty() || junctionProjection.overflowed)
            {
                supported=false;
                break;
            }
            for (uint64_t pixel : junctionProjection.pixels)
            {
                if (!native[d]->observed.pixels.contains(pixel)
                    || !projectedUses[d].contains(pixel))
                {
                    supported=false;
                    break;
                }
            }
        }
        if (!supported)
            continue;
        result.insert(result.end(),faces.begin(),faces.end());
    }
    return result;
}
