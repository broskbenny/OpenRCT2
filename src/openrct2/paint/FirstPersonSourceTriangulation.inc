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
        size_t originalFaceCount, size_t maximumFaces = 768)
{
    if (!hull.valid || views.size() != 4 || originalFaceCount < 200
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
            if (distinct.size() >= 80)
                break;
        }
        lines[d] = std::move(distinct);
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
    constexpr int pairs[4][2]={{0,1},{0,2},{1,3},{2,3}};
    const FirstPersonVec3 centre{
        hull.minForward+float(hull.sizeForward)*hull.step*.5f,
        hull.minRight+float(hull.sizeRight)*hull.step*.5f,
        hull.minUp+float(hull.sizeUp)*hull.step*.5f
    };
    for (const auto& pair : pairs)
    {
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

    const auto makeFaces = [&](FirstPersonVec3 start,FirstPersonVec3 end) {
        std::vector<FirstPersonVisualHullFace> out;
        const auto direction=unit(sub(end,start));
        const auto vertical=std::abs(direction.z)<.8f
            ? FirstPersonVec3{0,0,1} : FirstPersonVec3{0,1,0};
        const auto side=unit(FpCross(direction,vertical));
        const auto across=unit(FpCross(direction,side));
        constexpr float radius=.45f;
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
    std::vector<FirstPersonVisualHullFace> result;
    result.reserve(count*6);
    for (size_t i=0;i<candidates.size();++i)
        if (chosen[i])
            result.insert(result.end(),
                candidates[i].faces.begin(),candidates[i].faces.end());
    if (result.size()>maximumFaces
        || result.size()*3>=originalFaceCount*2)
        return {};
    return result;
}
