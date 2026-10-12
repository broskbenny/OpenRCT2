#include "FirstPersonConnectedPlanarMesh.h"
#include <iostream>
#include <fstream>
#include <iomanip>
struct Point {float x,y,z;};
int main(int argc,char**argv){
 if(argc!=3)return 1;
 std::ifstream in(argv[1],std::ios::binary);
 uint32_t nv=0,nf=0;in.read((char*)&nv,4);in.read((char*)&nf,4);
 if(!in || nv>100000 || nf>100000)return 2;
 std::vector<Point> v(nv);std::vector<OpenRCT2::Paint::FirstPersonTriangleIndices> f(nf);
 in.read((char*)v.data(),nv*sizeof(Point));in.read((char*)f.data(),nf*sizeof(f[0]));
 if(!in)return 2;
 auto [out,stats]=OpenRCT2::Paint::RetessellateFirstPersonConnectedPlanarMesh(v,f);
 std::ofstream file(argv[2],std::ios::binary);uint32_t count=out.size();file.write((char*)&count,4);file.write((char*)out.data(),out.size()*sizeof(out[0]));
 std::cout<<stats.originalTriangles<<" -> "<<stats.outputTriangles<<" triangles patches "<<stats.replacedPatches<<"\n";
}
