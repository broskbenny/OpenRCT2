int main() {
    // The production rasterizer has zero-area projection for these quads.
    // A thin rod's real source pixel must remain addressable, while large
    // or malformed spans must not receive the subpixel material exception.
    {
        const std::array<ScreenCoordsXY, 4> collapsed{{
            { 10, 20 }, { 11, 20 }, { 11, 20 }, { 10, 20 }
        }};
        FirstPersonSilhouette empty{};
        AddFirstPersonSilhouetteQuad(empty, collapsed);
        assert(empty.empty());
        const auto sample =
            FirstPersonSubpixelFaceSourceSample(collapsed);
        assert(sample.has_value() && sample->x == 11
            && sample->y == 20);
        const std::array<ScreenCoordsXY, 4> large{{
            { 10, 20 }, { 25, 20 }, { 25, 21 }, { 10, 21 }
        }};
        assert(!FirstPersonSubpixelFaceSourceSample(large).has_value());
    }
    // Source registration must also rescue a completely empty four-view
    // voxel intersection. All four sprites below depict the same valid
    // 16x16x24 source-backed cuboid, with independently shifted drawing
    // origins. No guessed/uncertified occupancy may become visible.
    for (const bool inset : { true, false })
    {
        SmallSceneryEntry entry{};
        entry.image = 700;
        entry.height = 32;
        entry.flags.set(SmallSceneryFlag::occupiesFullTile);
        entry.flags.set(SmallSceneryFlag::isRotatable);
        SmallSceneryElement element{};
        element.setOccupiedQuadrants(15);
        element.setDirection(0);
        const float lo = inset ? 8.0f : 0.0f;
        const float hi = inset ? 24.0f : 32.0f;
        const float bottom = inset ? 4.0f : 0.0f;
        const float top = inset ? 28.0f : 32.0f;
        const std::array<FirstPersonVec3, 8> p{{
            {lo, lo, bottom}, {hi, lo, bottom},
            {hi, hi, bottom}, {lo, hi, bottom},
            {lo, lo, top}, {hi, lo, top},
            {hi, hi, top}, {lo, hi, top}
        }};
        std::vector<FirstPersonVisualHullFace> cuboid;
        const auto side = [&](size_t a, size_t b, size_t c, size_t d) {
            cuboid.push_back({ { p[a], p[b], p[c], p[d] }, {}, {} });
        };
        side(0, 3, 2, 1);
        side(4, 5, 6, 7);
        side(0, 1, 5, 4);
        side(1, 2, 6, 5);
        side(2, 3, 7, 6);
        side(3, 0, 4, 7);
        const auto project = [&](uint8_t rotation, FirstPersonVec3 point) {
            const CoordsXYZ xyz{
                int32_t(std::lround(point.x)),
                int32_t(std::lround(point.y)),
                int32_t(std::lround(point.z)) };
            const auto uv = FirstPersonSmallSceneryArtworkPoint(
                entry, rotation, xyz);
            const auto offset = FirstPersonSmallSceneryPaintOffset(
                entry, element, rotation);
            const auto origin = Translate3DTo2DWithZ(rotation, { offset, 0 });
            return std::array<float, 2>{
                float(uv.x - origin.x), float(uv.y - origin.y) };
        };
        const auto tris = TriangulateFirstPersonReconstructionFaces(cuboid);
        FirstPersonSpriteSnapshot snapshot;
        FirstPersonFrameBudget budget(4, std::chrono::seconds(8));
        constexpr std::array<int32_t, 4> xShifts{{23, -23, 23, -23}};
        for (uint8_t rotation = 0; rotation < 4; ++rotation)
        {
            const auto source = RenderFirstPersonReconstructionSilhouette(
                tris, rotation, project);
            assert(source.size() > 100);
            G1Element sprite{};
            sprite.width = source.maxX - source.minX;
            sprite.height = source.maxY - source.minY;
            sprite.xOffset = source.minX + xShifts[rotation];
            sprite.yOffset = source.minY;
            sprite.flags.set(G1Flag::hasTransparency);
            std::vector<uint8_t> pixels(
                size_t(sprite.width) * size_t(sprite.height), 0);
            for (const auto key : source.pixels)
            {
                const int32_t x = FirstPersonSilhouettePixelX(key) - source.minX;
                const int32_t y = FirstPersonSilhouettePixelY(key) - source.minY;
                pixels[size_t(y) * size_t(sprite.width) + size_t(x)] = 180;
            }
            sprite.offset = pixels.data();
            assert(snapshot.capture(entry.image + rotation, &sprite, budget));
        }
        const auto hull = BuildFirstPersonSmallSceneryVisualHullFromSnapshot(
            entry, element, [&](ImageIndex id) { return snapshot.get(id); });
        assert(hull.valid && hull.roundTripViewCount == 4);
        assert(hull.attempt.registrationAttempted);
        for (uint8_t rotation = 0; rotation < 4; ++rotation)
            assert(std::abs(hull.viewPixelOffsets[rotation].x) >= 15);
        const auto appearance =
            BuildFirstPersonSmallSceneryAppearancePlan(entry, element, &hull);
        assert(appearance.valid() && !appearance.faces.empty());
        // Compact full-tile solids should never be turned into a highly
        // fragmented voxel mesh merely because sprite origins differ.
        if (!inset)
        {
            assert(hull.directConvexSurface);
            assert(appearance.faces.size() <= 6);
        }
        FirstPersonScene scene;
        FirstPersonVisualHullAttempt attempt{};
        SmallSceneryBuildEvidence evidence{};
        const bool built = BuildSmallSceneryGeometry(
            scene, element, entry, ImageId(entry.image), snapshot,
            &attempt, &evidence);
        assert(built && !scene.surfaces.empty());
        assert(attempt.registrationAttempted && attempt.certificateViews == 4);
        std::cout << "Four-view zero-overlap registration rescue passed"
                  << " inset=" << inset
                  << " directConvex=" << hull.directConvexSurface
                  << " faces=" << appearance.faces.size()
                  << " material=" << evidence.materialFaces << "\n";
    }
    for(int shape=0;shape<4;++shape) for (const float base : {0.0f, 4.0f}) {
        SmallSceneryEntry entry{};entry.image=100;entry.height=32;
        entry.flags.set(SmallSceneryFlag::occupiesFullTile);
        SmallSceneryElement element{};element.setOccupiedQuadrants(15);element.setDirection(0);
        std::vector<FirstPersonVisualHullFace> faces;
        const FirstPersonVec3 a{0,0,base},b{32,0,base},c{32,32,base},d{0,32,base};
        auto tri=[&](auto x,auto y,auto z){faces.push_back({{x,y,z,z},{},{}});};
        tri(a,c,b);tri(a,d,c);
        if(shape<2) {
            const FirstPersonVec3 top{shape==0?16.0f:10.0f,shape==0?16.0f:22.0f,24};
            tri(a,b,top);tri(b,c,top);tri(c,d,top);tri(d,a,top);
        } else if(shape==2) {
            FirstPersonVec3 e{0,0,24},f{32,0,24},g{32,32,24},h{0,32,24};
            tri(a,b,f);tri(a,f,e);tri(b,c,g);tri(b,g,f);tri(c,d,h);tri(c,h,g);tri(d,a,e);tri(d,e,h);tri(e,f,g);tri(e,g,h);
        } else {
            FirstPersonVec3 e{0,0,24},f{32,0,24};
            tri(a,b,f);tri(a,f,e);tri(b,c,f);tri(c,d,e);tri(c,e,f);tri(d,a,e);
        }
        const auto project=[&](uint8_t r,FirstPersonVec3 p){
            auto pixel=FirstPersonSmallSceneryArtworkPoint(entry,r,{int32_t(std::lround(p.x)),int32_t(std::lround(p.y)),int32_t(std::lround(p.z))});
            const auto offset=FirstPersonSmallSceneryPaintOffset(entry,element,r);
            const auto origin=Translate3DTo2DWithZ(r,{offset,0});
            return std::array<float,2>{float(pixel.x-origin.x),float(pixel.y-origin.y)};
        };
        FirstPersonSpriteSnapshot snapshot;
        FirstPersonFrameBudget budget(4,std::chrono::seconds(5));
        for(int r=0;r<4;++r){
            auto silhouette=RenderFirstPersonReconstructionSilhouette(TriangulateFirstPersonReconstructionFaces(faces),r,project);
            G1Element sprite{};sprite.width=silhouette.maxX-silhouette.minX;sprite.height=silhouette.maxY-silhouette.minY;
            sprite.xOffset=silhouette.minX;sprite.yOffset=silhouette.minY;sprite.flags.set(G1Flag::hasTransparency);
            std::vector<uint8_t> pixels(size_t(sprite.width)*sprite.height,0);
            for(const auto key:silhouette.pixels){const auto x=FirstPersonSilhouettePixelX(key),y=FirstPersonSilhouettePixelY(key);pixels[size_t(y-sprite.yOffset)*sprite.width+x-sprite.xOffset]=uint8_t(180+((x+y)&7));}
            sprite.offset=pixels.data();assert(snapshot.capture(100+r,&sprite,budget));
        }
        FirstPersonScene scene;FirstPersonVisualHullAttempt attempt;SmallSceneryBuildEvidence evidence;
        std::optional<FirstPersonSmallSceneryWalkingMask> walkingMask;
        auto start=std::chrono::steady_clock::now();
        bool good=BuildSmallSceneryGeometry(scene,element,entry,ImageId(100),snapshot,&attempt,&evidence,&walkingMask);
        auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        size_t missing=0,samples=0,texels=0;
        for(const auto& s:scene.surfaces){
            const int w=s.immutableWidth,h=s.immutableHeight;texels+=size_t(w)*h;
            FirstPersonVisualHullFace f;f.corners={s.triangles[0].world,s.triangles[1].world,s.triangles[2].world,s.triangles[5].world};
            for(int y=0;y<h;++y)for(int x=0;x<w;++x){
                const auto uv=FirstPersonVisualHullTexelUV(f,x,y,w,h);if(!uv)continue;
                ++samples;const auto pixel=(*s.persistentPixels)[size_t(y)*w+x];if(pixel==0)++missing;else assert(pixel>=180 && pixel<=187);
            }
        }
        assert(walkingMask.has_value() && walkingMask->valid);
        if(base==4) for(int y=0;y<16;++y) for(int x=0;x<16;++x) assert(!walkingMask->contains(0,x,y));
        assert(good && evidence.hullFaces<=6 && evidence.withoutSourceMaterial==0
            && evidence.withoutOpaqueSamples==0 && missing==0 && attempt.minimumIoU==1);
        std::cout<<"base="<<base<<" shape="<<shape<<" built="<<good<<" hullFaces="<<evidence.hullFaces<<" materialFaces="<<evidence.materialFaces<<" missingSource="<<evidence.withoutSourceMaterial<<" empty="<<evidence.withoutOpaqueSamples<<" underside="<<evidence.unobservedUndersides<<" transparent="<<missing<<"/"<<samples<<" texels="<<texels<<" IoU="<<attempt.minimumIoU<<" ms="<<ms<<"\n";
    }
    // A missing snapshot is unknown, whereas four transparent source views
    // provide actual evidence for an empty collision mask.
    for (int missingView = 0; missingView <= 4; ++missingView)
    {
        SmallSceneryEntry entry{};
        entry.image = 100;
        entry.height = 32;
        entry.flags.set(SmallSceneryFlag::isTree);
        SmallSceneryElement element{};
        element.setOccupiedQuadrants(15);
        std::array<uint8_t, 64 * 64> pixels{};
        G1Element sprite{};
        sprite.width = sprite.height = 64;
        sprite.xOffset = -32;
        sprite.yOffset = -48;
        sprite.offset = pixels.data();
        sprite.flags.set(G1Flag::hasTransparency);
        FirstPersonSpriteSnapshot snapshot;
        FirstPersonFrameBudget budget(4, std::chrono::seconds(5));
        for (int view = 0; view < 4; ++view)
            assert(snapshot.capture(100 + view, view == missingView ? nullptr : &sprite, budget));
        FirstPersonScene scene;
        std::optional<FirstPersonSmallSceneryWalkingMask> walkingMask;
        BuildSmallSceneryGeometry(scene, element, entry, ImageId(100), snapshot, nullptr, nullptr, &walkingMask);
        assert(walkingMask.has_value() == (missingView == 4));
        if (walkingMask) assert(!walkingMask->valid);
    }
    std::cout << "Missing-view and observed-empty collision publication checks passed\n";
    // Full production fixture: independent four-view source masks derived
    // from an open scaffold. Unlike the cuboid/roof cases above, no solid
    // convex envelope can explain its open regions. These are synthetic
    // palette indices, never proprietary RCT2 artwork.
    uint32_t unshiftedScaffoldMaterials = 0;
    for (const bool shiftedOrigins : { false, true })
    {
        SmallSceneryEntry entry{};
        entry.image = 200;
        entry.height = 48;
        entry.flags.set(SmallSceneryFlag::occupiesFullTile);
        entry.flags.set(SmallSceneryFlag::vOffsetCentre);
        entry.flags.set(SmallSceneryFlag::isRotatable);
        SmallSceneryElement element{};
        element.setOccupiedQuadrants(15);
        element.setDirection(0);
        FirstPersonVisualHull scaffold{};
        scaffold.valid = true;
        scaffold.step = 2;
        scaffold.sizeForward = 16;
        scaffold.sizeRight = 16;
        scaffold.sizeUp = 24;
        scaffold.occupied.assign(16 * 16 * 24, 0);
        const auto mark = [&](int f,int r,int u) {
            scaffold.occupied[(size_t(u)*16+size_t(r))*16+size_t(f)] = 1;
        };
        for (int u = 1; u < 23; ++u)
        for (int f : {1, 6, 11, 14})
        for (int r : {1, 14})
            mark(f, r, u);
        for (int f = 1; f < 15; ++f)
        for (int r : {1, 14})
        for (int u : {2, 9, 16, 22})
            mark(f, r, u);
        for (int f = 1; f < 15; ++f)
        for (int r : {1, 14}) {
            mark(f,r,f + 4);
            mark(f,r,19 - f);
        }
        const auto original = BuildFirstPersonVisualHullVoxelBoundaryFaces(scaffold);
        assert(original.size() >= 64);
        const auto triangles = TriangulateFirstPersonReconstructionFaces(original);
        const auto project = [&](uint8_t rotation, FirstPersonVec3 point) {
            const CoordsXYZ p{
                int32_t(std::lround(point.x)), int32_t(std::lround(point.y)),
                int32_t(std::lround(point.z)) };
            const auto uv = FirstPersonSmallSceneryArtworkPoint(entry, rotation, p);
            const auto offset = FirstPersonSmallSceneryPaintOffset(entry, element, rotation);
            const auto origin = Translate3DTo2DWithZ(rotation, { offset, 0 });
            return std::array<float,2>{
                float(uv.x - origin.x), float(uv.y - origin.y) };
        };
        FirstPersonSpriteSnapshot snapshot;
        FirstPersonFrameBudget budget(4, std::chrono::seconds(8));
        for (uint8_t rotation=0; rotation<4; ++rotation)
        {
            const auto mask = RenderFirstPersonReconstructionSilhouette(
                triangles, rotation, project);
            assert(mask.size() >= 250 && !mask.overflowed);
            G1Element g1{};
            g1.width = mask.maxX - mask.minX;
            g1.height = mask.maxY - mask.minY;
            // Independent native drawing origins must alter image-space
            // registration, not the world-space mesh or its G1 material.
            g1.xOffset = mask.minX
                + (shiftedOrigins ? ((rotation & 1u) ? -14 : 14) : 0);
            g1.yOffset = mask.minY;
            g1.flags.set(G1Flag::hasTransparency);
            std::vector<uint8_t> pixels(size_t(g1.width)*size_t(g1.height),0);
            for (uint64_t pixel : mask.pixels)
            {
                // The DATA in each G1 stays fixed; shifting only the
                // drawing origin is what the native-registration stage
                // must compensate for.
                const int32_t x=FirstPersonSilhouettePixelX(pixel)-mask.minX;
                const int32_t y=FirstPersonSilhouettePixelY(pixel)-mask.minY;
                pixels[size_t(y)*size_t(g1.width)+size_t(x)] =
                    uint8_t(150 + ((x+y)&7));
            }
            g1.offset = pixels.data();
            assert(snapshot.capture(entry.image+rotation,&g1,budget));
        }
        FirstPersonScene scene;
        FirstPersonVisualHullAttempt attempt{};
        SmallSceneryBuildEvidence evidence{};
        std::optional<FirstPersonSmallSceneryWalkingMask> mask{};
        const auto start=std::chrono::steady_clock::now();
        const bool built=BuildSmallSceneryGeometry(
            scene,element,entry,ImageId(entry.image),snapshot,
            &attempt,&evidence,&mask);
        const double elapsed=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-start).count();
        std::cout<<"scaffold built="<<built<<" faces="<<evidence.hullFaces
            <<" material="<<evidence.materialFaces
            <<" missing="<<evidence.withoutSourceMaterial
            <<" empty="<<evidence.withoutOpaqueSamples
            <<" methodWire="<<evidence.sourceTriangulatedWireSelected
            <<" stage="<<int(attempt.stage)<<" IoU="<<attempt.minimumIoU
            <<" ms="<<elapsed<<"\n";
        assert(mask.has_value() && mask->valid);
        std::cerr<<"SC_SOURCE_ORIGIN_TEST shifted="<<shiftedOrigins
            <<" built="<<built<<" surfaces="<<scene.surfaces.size()
            <<" faces="<<evidence.hullFaces
            <<" materials="<<evidence.materialFaces
            <<" missing="<<evidence.withoutSourceMaterial
            <<" empty="<<evidence.withoutOpaqueSamples
            <<" sourceOnly="<<evidence.sourceTriangulatedWireSelected
            <<" stage="<<int(attempt.stage)
            <<" cells="<<attempt.occupiedCells
            <<" IoU="<<attempt.minimumIoU
            <<" cand="<<attempt.candidateCoverage
            <<" obs="<<attempt.observedCoverage
            <<" reg="<<attempt.registrationAttempted
            <<" views="<<int(attempt.nativeViews)
            <<std::endl;
        assert(built && !scene.surfaces.empty());
        assert(evidence.materialFaces>0);
        assert(evidence.sourceTriangulatedWireSelected);
        if (!shiftedOrigins)
            unshiftedScaffoldMaterials=evidence.materialFaces;
        else
        {
            // Actual source-only reconstruction must now preserve the
            // registration shifts in the returned hull that drives G1
            // sampling, not just use the shifts while certifying a mesh.
            const auto registered = BuildFirstPersonSmallSceneryVisualHullFromSnapshot(
                entry, element, [&](ImageIndex id) { return snapshot.get(id); });
            assert(registered.sourceTriangulatedWireSelected);
            assert(registered.attempt.registrationAttempted);
            for(uint8_t rotation=0;rotation<4;++rotation)
                assert(std::abs(registered.viewPixelOffsets[rotation].x)>=7);
            assert(evidence.materialFaces*4>=unshiftedScaffoldMaterials*3);
        }
    }

}
