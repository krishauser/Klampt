#ifndef COLLIDE_H
#define COLLIDE_H

int newGeom();
void destroyGeom(int geom);

void makeTriMeshGeom(int geom,const char* fn);
void makeTriMeshGeom(int geom,const double* verts,const int* inds,int nv,int nt);
void setTriMeshTranslation(int geom,const double t[3]);
void setTriMeshRotation(int geom,const double r[9]);
void getTriMeshTranslation(int geom,double out[3]);
void getTriMeshRotation(int geom,double out[9]);
void getTriMeshBB(int geom,double out[3],double out2[3]);
int getTriMeshNumVerts(int geom);
int getTriMeshNumTris(int geom);
double* getTriMeshVerts(int geom);
int* getTriMeshTris(int geom);

void makePointGeom(int geom,const double x[3]);
void makeSphereGeom(int geom,const double c[3],double r);
void makeRayGeom(int geom,const double s[3],const double d[3]);
void makeLineGeom(int geom,const double s[3],const double d[3]);
void makeSegmentGeom(int geom,const double a[3],const double b[3]);
void makeAABBGeom(int geom,const double bmin[3],const double bmax[3]);
void makeGroupGeom(int geom,int* geoms,int numgeoms);

bool collide(int geom1,int geom2);
bool withinTolerance(int geom1,int geom2,double tol);
double distance(int geom1,int geom2,double relErr,double absErr);
void closestPoints(int geom1,int geom2,double out[3],double out2[3]);
bool rayCast(int geom,const double s[3],const double d[3],double out[3]);

int makeCollQuery(int geom1,int geom2);
void destroyCollQuery(int query);
bool queryCollide(int query);
bool queryWithinTolerance(int query,double tol);
double queryDistance(int query,double relErr,double absErr);
void queryClosestPoints(int query,double out[3],double out2[3]);
void queryTolerancePoints(int query,double out[3],double out2[3]);

void destroy();

#endif
