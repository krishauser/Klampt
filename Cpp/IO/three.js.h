#ifndef IO_THREE_JS_H
#define IO_THREE_JS_H

#include <Klampt/Modeling/World.h>
#include <Klampt/Simulation/Simulator.h>
#include <KrisLibrary/utils/AnyCollection.h>

namespace Klampt {

/** @addtogroup IO */
/*@{*/

///Exports a world to a JSON object that can be used in the three.js editor.
///Contains metadata, geometries, materials, and object items.
///
///Note: export the AnyCollection to a string via the ostream << operator.
void ThreeJSExport(const WorldModel& world,AnyCollection& out);
void ThreeJSExportTransforms(const WorldModel& world,AnyCollection& out);

///Exports a simulation to a JSON object that can be used in the three.js
///editor.  Contains metadata, geometries, materials, and object items.
///Draws the simulation world in natural color and the commanded
///world in transparent green.
void ThreeJSExport(Simulator& sim,AnyCollection& out);
void ThreeJSExportTransforms(Simulator& sim,AnyCollection& out);

///Exports a robot to a JSON object that can be used in the three.js editor.
///The result is a hierarchical set of Mesh or Group objects.
void ThreeJSExport(const RobotModel& robot,AnyCollection& out);
void ThreeJSExportTransforms(const RobotModel& robot,AnyCollection& out);

///Exports a rigid object to a JSON object that can be used in the three.js
///editor. The result is a Mesh object (or Group if the geometry is empty).
void ThreeJSExport(const RigidObjectModel& object,AnyCollection& out);
void ThreeJSExportTransforms(const RigidObjectModel& object,AnyCollection& out);

///Exports a rigid object to a JSON object that can be used in the three.js
///editor. The result is a Mesh object (or Group if the geometry is empty).
void ThreeJSExport(const TerrainModel& terrain,AnyCollection& out);
void ThreeJSExportTransforms(const TerrainModel& terrain,AnyCollection& out);

///Exports geometry to a three.js scene Mesh / Points instance. Preferable to exporting Geometry and Appearance separately.
void ThreeJSExport(const ManagedGeometry& geom,AnyCollection& out);
///Exports geometry to a three.js scene Geometry instance.  The "uuid"
///element of the output gives the unique ID number that can be used elsewhere
void ThreeJSExportGeometry(const ManagedGeometry& geom,AnyCollection& out);
///Exports appearance to a three.js scene Material instance.  The "uuid"
///element of the output gives the unique ID number that can be used elsewhere
void ThreeJSExportAppearance(const ManagedGeometry& geom,AnyCollection& out);
///Exports to a three.js scene Geometry instance
void ThreeJSExport(const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out);
///Exports to a three.js scene Material instance
void ThreeJSExport(const GLDraw::GeometryAppearance& app,const Geometry::AnyCollisionGeometry3D& geom,AnyCollection& out);

/*@}*/

} // namespace Klampt

#endif
