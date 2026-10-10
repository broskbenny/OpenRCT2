int main() {
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
}
