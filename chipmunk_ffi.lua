--====================================================================
local ffi           = require("ffi")
local TargetDllName = "chipmunk-7.0.1.dll"
--====================================================================
-- chipmunk.h
ffi.cdef([[
void cpMessage( const char *condition
              , const char *file
              , int line, int isError, int isHardError, const char *message, ...);

typedef struct cpArray              cpArray;
typedef struct cpHashSet            cpHashSet;
typedef struct cpBody               cpBody;
typedef struct cpShape              cpShape;
typedef struct cpCircleShape        cpCircleShape;
typedef struct cpSegmentShape       cpSegmentShape;
typedef struct cpPolyShape          cpPolyShape;
typedef struct cpConstraint         cpConstraint;
typedef struct cpPinJoint           cpPinJoint;
typedef struct cpSlideJoint         cpSlideJoint;
typedef struct cpPivotJoint         cpPivotJoint;
typedef struct cpGrooveJoint        cpGrooveJoint;
typedef struct cpDampedSpring       cpDampedSpring;
typedef struct cpDampedRotarySpring cpDampedRotarySpring;
typedef struct cpRotaryLimitJoint   cpRotaryLimitJoint;
typedef struct cpRatchetJoint       cpRatchetJoint;
typedef struct cpGearJoint          cpGearJoint;
typedef struct cpSimpleMotorJoint   cpSimpleMotorJoint;
typedef struct cpCollisionHandler   cpCollisionHandler;
typedef struct cpContactPointSet    cpContactPointSet;
typedef struct cpArbiter            cpArbiter;
typedef struct cpSpace              cpSpace;

// Chipmunk 7.0.1
enum
{
 CP_VERSION_MAJOR =7
,CP_VERSION_MINOR =0
,CP_VERSION_RELEASE1
};
]])
--====================================================================
-- chipmunk_types.h
ffi.cdef([[
typedef double  cpFloat;
// typedef float cpFloat;
// typedef CP_HASH_VALUE_TYPE cpHashValue;

typedef uintptr_t     cpHashValue;
typedef uint32_t      cpCollisionID;
typedef unsigned char cpBool;
typedef void*         cpDataPointer;
typedef uintptr_t     cpCollisionType;
typedef uintptr_t     cpGroup;
typedef unsigned int  cpBitmask;
typedef unsigned int  cpTimestamp;
typedef struct        cpVect{cpFloat x,y;} cpVect;
typedef struct        cpTransform
{
    cpFloat a, b, c, d, tx, ty;
} cpTransform;
// NUKE
typedef struct cpMat2x2 {
    // Row major [\[a, b][c d]\]
    cpFloat a, b, c, d;
} cpMat2x2;
]])
--====================================================================
-- cpVect.h
ffi.cdef([[]])
--====================================================================
-- cpBB.h  to chipmunk_ffi.h
ffi.cdef([[
/// Chipmunk's axis-aligned 2D bounding box type. (left, bottom, right, top)
typedef struct cpBB{
    cpFloat l, b, r ,t;
} cpBB;

/// Convenience constructor for cpBB structs.
static inline cpBB cpBBNew(const cpFloat l, const cpFloat b, const cpFloat r, const cpFloat t)
{
    cpBB bb = {l, b, r, t};
    return bb;
}
]])
--====================================================================
-- cpTransform.h to chipmunk_ffi.h
ffi.cdef([[]])
--====================================================================
-- cpSpatialIndex.h
ffi.cdef([[
typedef cpBB          (*cpSpatialIndexBBFunc)           ( void *obj);
typedef void          (*cpSpatialIndexIteratorFunc)     ( void *obj, void *data);
typedef cpCollisionID (*cpSpatialIndexQueryFunc)        ( void *obj1, void *obj2, cpCollisionID id, void *data);
typedef cpFloat       (*cpSpatialIndexSegmentQueryFunc) ( void *obj1, void *obj2, void *data);

typedef struct cpSpatialIndexClass cpSpatialIndexClass;
typedef struct cpSpatialIndex cpSpatialIndex;
typedef struct cpSpaceHash cpSpaceHash;

/// @private
struct cpSpatialIndex {
    cpSpatialIndexClass *klass;
    cpSpatialIndexBBFunc bbfunc;
    cpSpatialIndex *staticIndex, *dynamicIndex;
};

//MARK: Spatial Hash
typedef struct  cpSpaceHash cpSpaceHash;
cpSpaceHash*    cpSpaceHashAlloc                 ( void);
cpSpatialIndex* cpSpaceHashInit                  ( cpSpaceHash *hash, cpFloat celldim, int numcells, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex);
cpSpatialIndex* cpSpaceHashNew                   ( cpFloat celldim, int cells, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex);
void            cpSpaceHashResize                ( cpSpaceHash *hash, cpFloat celldim, int numcells);

//MARK: AABB Tree
typedef struct   cpBBTree cpBBTree;
cpBBTree*        cpBBTreeAlloc                   ( void);
cpSpatialIndex*  cpBBTreeInit                    ( cpBBTree *tree, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex);
cpSpatialIndex*  cpBBTreeNew                     ( cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex);
void             cpBBTreeOptimize                ( cpSpatialIndex *index);
typedef cpVect (*cpBBTreeVelocityFunc)           ( void *obj);
void             cpBBTreeSetVelocityFunc         ( cpSpatialIndex *index, cpBBTreeVelocityFunc func);

//MARK: Single Axis Sweep
typedef struct  cpSweep1D cpSweep1D;
cpSweep1D*      cpSweep1DAlloc                   ( void);
cpSpatialIndex* cpSweep1DInit                    ( cpSweep1D *sweep, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex);
cpSpatialIndex* cpSweep1DNew                     ( cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex);

//MARK: Spatial Index Implementation
typedef void   (*cpSpatialIndexDestroyImpl)      ( cpSpatialIndex *index);
typedef int    (*cpSpatialIndexCountImpl)        ( cpSpatialIndex *index);
typedef void   (*cpSpatialIndexEachImpl)         ( cpSpatialIndex *index, cpSpatialIndexIteratorFunc func, void *data);
typedef cpBool (*cpSpatialIndexContainsImpl)     ( cpSpatialIndex *index, void *obj, cpHashValue hashid);
typedef void   (*cpSpatialIndexInsertImpl)       ( cpSpatialIndex *index, void *obj, cpHashValue hashid);
typedef void   (*cpSpatialIndexRemoveImpl)       ( cpSpatialIndex *index, void *obj, cpHashValue hashid);
typedef void   (*cpSpatialIndexReindexImpl)      ( cpSpatialIndex *index);
typedef void   (*cpSpatialIndexReindexObjectImpl)( cpSpatialIndex *index, void *obj, cpHashValue hashid);
typedef void   (*cpSpatialIndexReindexQueryImpl) ( cpSpatialIndex *index, cpSpatialIndexQueryFunc func, void *data);
typedef void   (*cpSpatialIndexQueryImpl)        ( cpSpatialIndex *index, void *obj, cpBB bb, cpSpatialIndexQueryFunc func, void *data);
typedef void   (*cpSpatialIndexSegmentQueryImpl) ( cpSpatialIndex *index, void *obj, cpVect a, cpVect b, cpFloat t_exit, cpSpatialIndexSegmentQueryFunc func, void *data);

struct cpSpatialIndexClass {
    cpSpatialIndexDestroyImpl       destroy;
    cpSpatialIndexCountImpl         count;
    cpSpatialIndexEachImpl          each;
    cpSpatialIndexContainsImpl      contains;
    cpSpatialIndexInsertImpl        insert;
    cpSpatialIndexRemoveImpl        remove;
    cpSpatialIndexReindexImpl       reindex;
    cpSpatialIndexReindexObjectImpl reindexObject;
    cpSpatialIndexReindexQueryImpl  reindexQuery;
    cpSpatialIndexQueryImpl         query;
    cpSpatialIndexSegmentQueryImpl  segmentQuery;
};

]])
--====================================================================
-- cpArbiter.h
ffi.cdef([[
enum
{
CP_MAX_CONTACTS_PER_ARBITER = 2
};

cpFloat       cpArbiterGetRestitution     ( const cpArbiter *arb);
void          cpArbiterSetRestitution     ( cpArbiter *arb, cpFloat restitution);
cpFloat       cpArbiterGetFriction        ( const cpArbiter *arb);
void          cpArbiterSetFriction        ( cpArbiter *arb, cpFloat friction);
cpVect        cpArbiterGetSurfaceVelocity ( cpArbiter *arb);
void          cpArbiterSetSurfaceVelocity ( cpArbiter *arb, cpVect vr);
cpDataPointer cpArbiterGetUserData        ( const cpArbiter *arb);
void          cpArbiterSetUserData        ( cpArbiter *arb, cpDataPointer userData);
cpVect        cpArbiterTotalImpulse       ( const cpArbiter *arb);
cpFloat       cpArbiterTotalKE            ( const cpArbiter *arb);
cpBool        cpArbiterIgnore             ( cpArbiter *arb);
void          cpArbiterGetShapes          ( const cpArbiter *arb, cpShape **a, cpShape **b);

// A macro shortcut for defining and retrieving the shapes from an arbiter.
//#define CP_ARBITER_GET_SHAPES(__arb__, __a__, __b__) cpShape *__a__, *__b__; cpArbiterGetShapes(__arb__, &__a__, &__b__);
void          cpArbiterGetBodies          (const cpArbiter *arb, cpBody **a, cpBody **b);

/// A macro shortcut for defining and retrieving the bodies from an arbiter.
//#define CP_ARBITER_GET_BODIES(__arb__, __a__, __b__) cpBody *__a__, *__b__; cpArbiterGetBodies(__arb__, &__a__, &__b__);

struct cpContactPointSet
{
    int      count; /// The number of contact points in the set.
    cpVect   normal;/// The normal of the collision.
    struct {        /// The array of contact points.
        /// The position of the contact on the surface of each shape.
        cpVect pointA, pointB;
        /// Penetration distance of the two shapes. Overlapping means it will be negative.
        /// This value is calculated as cpvdot(cpvsub(point2, point1), normal) and is ignored by cpArbiterSetContactPointSet().
        cpFloat distance;
    } points[CP_MAX_CONTACTS_PER_ARBITER];
};

cpContactPointSet cpArbiterGetContactPointSet ( const cpArbiter *arb);
void    cpArbiterSetContactPointSet           ( cpArbiter *arb, cpContactPointSet *set);
cpBool  cpArbiterIsFirstContact               ( const cpArbiter *arb);
cpBool  cpArbiterIsRemoval                    ( const cpArbiter *arb);
int     cpArbiterGetCount                     ( const cpArbiter *arb);
cpVect  cpArbiterGetNormal                    ( const cpArbiter *arb);
cpVect  cpArbiterGetPointA                    ( const cpArbiter *arb, int i);
cpVect  cpArbiterGetPointB                    ( const cpArbiter *arb, int i);
cpFloat cpArbiterGetDepth                     ( const cpArbiter *arb, int i);
cpBool  cpArbiterCallWildcardBeginA           ( cpArbiter *arb, cpSpace *space);
cpBool  cpArbiterCallWildcardBeginB           ( cpArbiter *arb, cpSpace *space);
cpBool  cpArbiterCallWildcardPreSolveA        ( cpArbiter *arb, cpSpace *space);
cpBool  cpArbiterCallWildcardPreSolveB        ( cpArbiter *arb, cpSpace *space);
void    cpArbiterCallWildcardPostSolveA       ( cpArbiter *arb, cpSpace *space);
void    cpArbiterCallWildcardPostSolveB       ( cpArbiter *arb, cpSpace *space);
void    cpArbiterCallWildcardSeparateA        ( cpArbiter *arb, cpSpace *space);
void    cpArbiterCallWildcardSeparateB        ( cpArbiter *arb, cpSpace *space);
]])
--====================================================================
-- cpBody.h
ffi.cdef([[
typedef enum cpBodyType {
    CP_BODY_TYPE_DYNAMIC,
    CP_BODY_TYPE_KINEMATIC,
    CP_BODY_TYPE_STATIC,
} cpBodyType;

typedef void (*cpBodyVelocityFunc)            ( cpBody *body, cpVect gravity, cpFloat damping, cpFloat dt);
typedef void (*cpBodyPositionFunc)            ( cpBody *body, cpFloat dt);

cpBody*        cpBodyAlloc                    ( void);
cpBody*        cpBodyInit                     ( cpBody *body, cpFloat mass, cpFloat moment);
cpBody*        cpBodyNew                      ( cpFloat mass, cpFloat moment);
cpBody*        cpBodyNewKinematic             ( void);
cpBody*        cpBodyNewStatic                ( void);
//====================================================================
void           cpBodyDestroy                  ( cpBody *body);
void           cpBodyFree                     ( cpBody *body);
void           cpBodyActivate                 ( cpBody *body);
void           cpBodyActivateStatic           ( cpBody *body, cpShape *filter);
void           cpBodySleep                    ( cpBody *body);
void           cpBodySleepWithGroup           ( cpBody *body, cpBody *group);
cpBool         cpBodyIsSleeping               ( const cpBody *body);
cpBodyType     cpBodyGetType                  ( cpBody *body);
void           cpBodySetType                  ( cpBody *body, cpBodyType type);
cpSpace*       cpBodyGetSpace                 ( const cpBody *body);
cpFloat        cpBodyGetMass                  ( const cpBody *body);
void           cpBodySetMass                  ( cpBody *body, cpFloat m);
cpFloat        cpBodyGetMoment                ( const cpBody *body);
void           cpBodySetMoment                ( cpBody *body, cpFloat i);
cpVect         cpBodyGetPosition              ( const cpBody *body);
void           cpBodySetPosition              ( cpBody *body, cpVect pos);
cpVect         cpBodyGetCenterOfGravity       ( const cpBody *body);
void           cpBodySetCenterOfGravity       ( cpBody *body, cpVect cog);
cpVect         cpBodyGetVelocity              ( const cpBody *body);
void           cpBodySetVelocity              ( cpBody *body, cpVect velocity);
cpVect         cpBodyGetForce                 ( const cpBody *body);
void           cpBodySetForce                 ( cpBody *body, cpVect force);
cpFloat        cpBodyGetAngle                 ( const cpBody *body);
void           cpBodySetAngle                 ( cpBody *body, cpFloat a);
cpFloat        cpBodyGetAngularVelocity       ( const cpBody *body);
void           cpBodySetAngularVelocity       ( cpBody *body, cpFloat angularVelocity);
cpFloat        cpBodyGetTorque                ( const cpBody *body);
void           cpBodySetTorque                ( cpBody *body, cpFloat torque);
cpVect         cpBodyGetRotation              ( const cpBody *body);
cpDataPointer  cpBodyGetUserData              ( const cpBody *body);
void           cpBodySetUserData              ( cpBody *body, cpDataPointer userData);
void           cpBodySetVelocityUpdateFunc    ( cpBody *body, cpBodyVelocityFunc velocityFunc);
void           cpBodySetPositionUpdateFunc    ( cpBody *body, cpBodyPositionFunc positionFunc);
void           cpBodyUpdateVelocity           ( cpBody *body, cpVect gravity, cpFloat damping, cpFloat dt);
void           cpBodyUpdatePosition           ( cpBody *body, cpFloat dt);
cpVect         cpBodyLocalToWorld             ( const cpBody *body, const cpVect point);
cpVect         cpBodyWorldToLocal             ( const cpBody *body, const cpVect point);
void           cpBodyApplyForceAtWorldPoint   ( cpBody *body, cpVect force, cpVect point);
void           cpBodyApplyForceAtLocalPoint   ( cpBody *body, cpVect force, cpVect point);
void           cpBodyApplyImpulseAtWorldPoint ( cpBody *body, cpVect impulse, cpVect point);
void           cpBodyApplyImpulseAtLocalPoint ( cpBody *body, cpVect impulse, cpVect point);
cpVect         cpBodyGetVelocityAtWorldPoint  ( const cpBody *body, cpVect point);
cpVect         cpBodyGetVelocityAtLocalPoint  ( const cpBody *body, cpVect point);
cpFloat        cpBodyKineticEnergy            ( const cpBody *body);

typedef void (*cpBodyShapeIteratorFunc)       ( cpBody *body, cpShape *shape, void *data);
typedef void (*cpBodyArbiterIteratorFunc)     ( cpBody *body, cpArbiter *arbiter, void *data);
typedef void (*cpBodyConstraintIteratorFunc)  ( cpBody *body, cpConstraint *constraint, void *data);

void           cpBodyEachShape                ( cpBody *body, cpBodyShapeIteratorFunc func, void *data);
void           cpBodyEachConstraint           ( cpBody *body, cpBodyConstraintIteratorFunc func, void *data);
void           cpBodyEachArbiter              ( cpBody *body, cpBodyArbiterIteratorFunc func, void *data);
]])
--====================================================================
-- cpShape.h
ffi.cdef([[
typedef struct cpPointQueryInfo{
    const cpShape *shape;
    cpVect point;
    cpFloat distance;
    cpVect gradient;
} cpPointQueryInfo;

typedef struct cpSegmentQueryInfo {
    const cpShape *shape;
    cpVect point;
    cpVect normal;
    cpFloat alpha;
} cpSegmentQueryInfo;

typedef struct cpShapeFilter {
    cpGroup group;
    cpBitmask categories;
    cpBitmask mask;
} cpShapeFilter;

void                      cpShapeDestroy             ( cpShape *shape);
void                      cpShapeFree                ( cpShape *shape);
cpBB                      cpShapeCacheBB             ( cpShape *shape);
cpBB                      cpShapeGetBB               ( const cpShape *shape);
cpBB                      cpShapeUpdate              ( cpShape *shape, cpTransform transform);
cpFloat                   cpShapePointQuery          ( const cpShape *shape, cpVect p, cpPointQueryInfo *out);
cpBool                    cpShapeSegmentQuery        ( const cpShape *shape, cpVect a, cpVect b, cpFloat radius, cpSegmentQueryInfo *info);
cpContactPointSet         cpShapesCollide            ( const cpShape *a, const cpShape *b);
cpSpace*                  cpShapeGetSpace            ( const cpShape *shape);
cpBody*                   cpShapeGetBody             ( const cpShape *shape);
void                      cpShapeSetBody             ( cpShape *shape, cpBody *body);
// cpFloat                cpShapeGetMass             ( cpShape *shape); // ?

void                      cpShapeSetMass             ( cpShape *shape, cpFloat mass);
cpFloat                   cpShapeGetDensity          ( cpShape *shape);
void                      cpShapeSetDensity          ( cpShape *shape, cpFloat density);
cpFloat                   cpShapeGetMoment           ( cpShape *shape);
cpFloat                   cpShapeGetArea             ( cpShape *shape);
cpVect                    cpShapeGetCenterOfGravity  ( cpShape *shape);
cpBool                    cpShapeGetSensor           ( const cpShape *shape);
void                      cpShapeSetSensor           ( cpShape *shape, cpBool sensor);
cpFloat                   cpShapeGetElasticity       ( const cpShape *shape);
void                      cpShapeSetElasticity       ( cpShape *shape, cpFloat elasticity);
cpFloat                   cpShapeGetFriction         ( const cpShape *shape);
void                      cpShapeSetFriction         ( cpShape *shape, cpFloat friction);
cpVect                    cpShapeGetSurfaceVelocity  ( const cpShape *shape);
void                      cpShapeSetSurfaceVelocity  ( cpShape *shape, cpVect surfaceVelocity);
cpDataPointer             cpShapeGetUserData         ( const cpShape *shape);
void                      cpShapeSetUserData         ( cpShape *shape, cpDataPointer userData);
cpCollisionType           cpShapeGetCollisionType    ( const cpShape *shape);
void                      cpShapeSetCollisionType    ( cpShape *shape, cpCollisionType collisionType);
cpShapeFilter             cpShapeGetFilter           ( const cpShape *shape);
void                      cpShapeSetFilter           ( cpShape *shape, cpShapeFilter filter);

cpCircleShape*            cpCircleShapeAlloc         ( void);
cpCircleShape*            cpCircleShapeInit          ( cpCircleShape *circle, cpBody *body, cpFloat radius, cpVect offset);
cpShape*                  cpCircleShapeNew           ( cpBody *body, cpFloat radius, cpVect offset);
cpVect                    cpCircleShapeGetOffset     ( const cpShape *shape);
cpFloat                   cpCircleShapeGetRadius     ( const cpShape *shape);

cpSegmentShape*           cpSegmentShapeAlloc        ( void);
cpSegmentShape*           cpSegmentShapeInit         ( cpSegmentShape *seg, cpBody *body, cpVect a, cpVect b, cpFloat radius);
cpShape*                  cpSegmentShapeNew          ( cpBody *body, cpVect a, cpVect b, cpFloat radius);
void                      cpSegmentShapeSetNeighbors ( cpShape *shape, cpVect prev, cpVect next);
cpVect                    cpSegmentShapeGetA         ( const cpShape *shape);
cpVect                    cpSegmentShapeGetB         ( const cpShape *shape);
cpVect                    cpSegmentShapeGetNormal    ( const cpShape *shape);
cpFloat                   cpSegmentShapeGetRadius    ( const cpShape *shape);
]])
--====================================================================
-- cpPolyShape.h
ffi.cdef([[
cpPolyShape*              cpPolyShapeAlloc     ( void);
cpPolyShape*              cpPolyShapeInit      ( cpPolyShape *poly, cpBody *body, int count, const cpVect *verts, cpTransform transform, cpFloat radius);
cpPolyShape*              cpPolyShapeInitRaw   ( cpPolyShape *poly, cpBody *body, int count, const cpVect *verts, cpFloat radius);
cpShape*                  cpPolyShapeNew       ( cpBody *body, int count, const cpVect *verts, cpTransform transform, cpFloat radius);
cpShape*                  cpPolyShapeNewRaw    ( cpBody *body, int count, const cpVect *verts, cpFloat radius);
cpPolyShape*              cpBoxShapeInit       ( cpPolyShape *poly, cpBody *body, cpFloat width, cpFloat height, cpFloat radius);
cpPolyShape*              cpBoxShapeInit2      ( cpPolyShape *poly, cpBody *body, cpBB box, cpFloat radius);
cpShape*                  cpBoxShapeNew        ( cpBody *body, cpFloat width, cpFloat height, cpFloat radius);
cpShape*                  cpBoxShapeNew2       ( cpBody *body, cpBB box, cpFloat radius);
int                       cpPolyShapeGetCount  ( const cpShape *shape);
cpVect                    cpPolyShapeGetVert   ( const cpShape *shape, int index);
cpFloat                   cpPolyShapeGetRadius ( const cpShape *shape);
]])
--====================================================================
-- cpConstraint.h
ffi.cdef([[
typedef void (*cpConstraintPreSolveFunc )              ( cpConstraint *constraint, cpSpace *space);
typedef void (*cpConstraintPostSolveFunc)              ( cpConstraint *constraint, cpSpace *space);
void                      cpConstraintDestroy          ( cpConstraint *constraint);
void                      cpConstraintFree             ( cpConstraint *constraint);
cpSpace*                  cpConstraintGetSpace         ( const cpConstraint *constraint);
cpBody*                   cpConstraintGetBodyA         ( const cpConstraint *constraint);
cpBody*                   cpConstraintGetBodyB         ( const cpConstraint *constraint);
cpFloat                   cpConstraintGetMaxForce      ( const cpConstraint *constraint);
void                      cpConstraintSetMaxForce      ( cpConstraint *constraint, cpFloat maxForce);
cpFloat                   cpConstraintGetErrorBias     ( const cpConstraint *constraint);
void                      cpConstraintSetErrorBias     ( cpConstraint *constraint, cpFloat errorBias);
cpFloat                   cpConstraintGetMaxBias       ( const cpConstraint *constraint);
void                      cpConstraintSetMaxBias       ( cpConstraint *constraint, cpFloat maxBias);
cpBool                    cpConstraintGetCollideBodies ( const cpConstraint *constraint);
void                      cpConstraintSetCollideBodies ( cpConstraint *constraint, cpBool collideBodies);
cpConstraintPreSolveFunc  cpConstraintGetPreSolveFunc  ( const cpConstraint *constraint);
void                      cpConstraintSetPreSolveFunc  ( cpConstraint *constraint, cpConstraintPreSolveFunc preSolveFunc);
cpConstraintPostSolveFunc cpConstraintGetPostSolveFunc ( const cpConstraint *constraint);
void                      cpConstraintSetPostSolveFunc ( cpConstraint *constraint, cpConstraintPostSolveFunc postSolveFunc);
cpDataPointer             cpConstraintGetUserData      ( const cpConstraint *constraint);
void                      cpConstraintSetUserData      ( cpConstraint *constraint, cpDataPointer userData);
cpFloat                   cpConstraintGetImpulse       ( cpConstraint *constraint);
]])
--====================================================================
--#include "cpPinJoint.h" from cpConstraint.h
ffi.cdef([[
cpBool                    cpConstraintIsPinJoint       ( const cpConstraint *constraint);
cpPinJoint*               cpPinJointAlloc              ( void);
cpPinJoint*               cpPinJointInit               ( cpPinJoint *joint, cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB);
cpConstraint*             cpPinJointNew                ( cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB);
cpVect                    cpPinJointGetAnchorA         ( const cpConstraint *constraint);
void                      cpPinJointSetAnchorA         ( cpConstraint *constraint, cpVect anchorA);
cpVect                    cpPinJointGetAnchorB         ( const cpConstraint *constraint);
void                      cpPinJointSetAnchorB         ( cpConstraint *constraint, cpVect anchorB);
cpFloat                   cpPinJointGetDist            ( const cpConstraint *constraint);
void                      cpPinJointSetDist            ( cpConstraint *constraint, cpFloat dist);
]])
--====================================================================
--#include "cpSlideJoint.h" from cpConstraint.h
ffi.cdef([[
cpBool                    cpConstraintIsSlideJoint         ( const cpConstraint *constraint);
cpSlideJoint*             cpSlideJointAlloc                ( void);
cpSlideJoint*             cpSlideJointInit                 ( cpSlideJoint *joint, cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB, cpFloat min, cpFloat max);
cpConstraint*             cpSlideJointNew                  ( cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB, cpFloat min, cpFloat max);
cpVect                    cpSlideJointGetAnchorA           ( const cpConstraint *constraint);
void                      cpSlideJointSetAnchorA           ( cpConstraint *constraint, cpVect anchorA);
cpVect                    cpSlideJointGetAnchorB           ( const cpConstraint *constraint);
void                      cpSlideJointSetAnchorB           ( cpConstraint *constraint, cpVect anchorB);
cpFloat                   cpSlideJointGetMin               ( const cpConstraint *constraint);
void                      cpSlideJointSetMin               ( cpConstraint *constraint, cpFloat min);
cpFloat                   cpSlideJointGetMax               ( const cpConstraint *constraint);
void                      cpSlideJointSetMax               ( cpConstraint *constraint, cpFloat max);
]])
--====================================================================
--#include "cpPivotJoint.h" from cpConstraint.h
ffi.cdef([[
cpBool                    cpConstraintIsPivotJoint         ( const cpConstraint *constraint);
cpPivotJoint*             cpPivotJointAlloc                ( void);
cpPivotJoint*             cpPivotJointInit                 ( cpPivotJoint *joint, cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB);
cpConstraint*             cpPivotJointNew                  ( cpBody *a, cpBody *b, cpVect pivot);
cpConstraint*             cpPivotJointNew2                 ( cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB);
cpVect                    cpPivotJointGetAnchorA           ( const cpConstraint *constraint);
void                      cpPivotJointSetAnchorA           ( cpConstraint *constraint, cpVect anchorA);
cpVect                    cpPivotJointGetAnchorB           ( const cpConstraint *constraint);
void                      cpPivotJointSetAnchorB           ( cpConstraint *constraint, cpVect anchorB);
]])
--====================================================================
--#include "cpGrooveJoint.h" from cpConstraint.h
ffi.cdef([[
cpBool                    cpConstraintIsGrooveJoint        ( const cpConstraint *constraint);
cpGrooveJoint*            cpGrooveJointAlloc               ( void);
cpGrooveJoint*            cpGrooveJointInit                ( cpGrooveJoint *joint, cpBody *a, cpBody *b, cpVect groove_a, cpVect groove_b, cpVect anchorB);
cpConstraint*             cpGrooveJointNew                 ( cpBody *a, cpBody *b, cpVect groove_a, cpVect groove_b, cpVect anchorB);
cpVect                    cpGrooveJointGetGrooveA          ( const cpConstraint *constraint);
void                      cpGrooveJointSetGrooveA          ( cpConstraint *constraint, cpVect grooveA);
cpVect                    cpGrooveJointGetGrooveB          ( const cpConstraint *constraint);
void                      cpGrooveJointSetGrooveB          ( cpConstraint *constraint, cpVect grooveB);
cpVect                    cpGrooveJointGetAnchorB          ( const cpConstraint *constraint);
void                      cpGrooveJointSetAnchorB          ( cpConstraint *constraint, cpVect anchorB);
]])
--====================================================================
--#include "cpDampedSpring.h" from cpConstraint.h
ffi.cdef([[
typedef cpFloat         (*cpDampedSpringForceFunc)         ( cpConstraint *spring, cpFloat dist);
cpBool                    cpConstraintIsDampedSpring       ( const cpConstraint *constraint);
cpDampedSpring*           cpDampedSpringAlloc              ( void);
cpDampedSpring*           cpDampedSpringInit               ( cpDampedSpring *joint, cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB, cpFloat restLength, cpFloat stiffness, cpFloat damping);
cpConstraint*             cpDampedSpringNew                ( cpBody *a, cpBody *b, cpVect anchorA, cpVect anchorB, cpFloat restLength, cpFloat stiffness, cpFloat damping);
cpVect                    cpDampedSpringGetAnchorA         ( const cpConstraint *constraint);
void                      cpDampedSpringSetAnchorA         ( cpConstraint *constraint, cpVect anchorA);
cpVect                    cpDampedSpringGetAnchorB         ( const cpConstraint *constraint);
void                      cpDampedSpringSetAnchorB         ( cpConstraint *constraint, cpVect anchorB);
cpFloat                   cpDampedSpringGetRestLength      ( const cpConstraint *constraint);
void                      cpDampedSpringSetRestLength      ( cpConstraint *constraint, cpFloat restLength);
cpFloat                   cpDampedSpringGetStiffness       ( const cpConstraint *constraint);
void                      cpDampedSpringSetStiffness       ( cpConstraint *constraint, cpFloat stiffness);
cpFloat                   cpDampedSpringGetDamping         ( const cpConstraint *constraint);
void                      cpDampedSpringSetDamping         ( cpConstraint *constraint, cpFloat damping);
cpDampedSpringForceFunc   cpDampedSpringGetSpringForceFunc ( const cpConstraint *constraint);
void                      cpDampedSpringSetSpringForceFunc ( cpConstraint *constraint, cpDampedSpringForceFunc springForceFunc);
]])
--====================================================================
--#include "cpDampedRotarySpring.h" from cpConstraint.h
ffi.cdef([[
typedef cpFloat              (*cpDampedRotarySpringTorqueFunc)         ( struct cpConstraint *spring, cpFloat relativeAngle);
cpBool                         cpConstraintIsDampedRotarySpring        ( const cpConstraint *constraint);
cpDampedRotarySpring*          cpDampedRotarySpringAlloc               ( void);
cpDampedRotarySpring*          cpDampedRotarySpringInit                ( cpDampedRotarySpring *joint, cpBody *a, cpBody *b, cpFloat restAngle, cpFloat stiffness, cpFloat damping);
cpConstraint*                  cpDampedRotarySpringNew                 ( cpBody *a, cpBody *b, cpFloat restAngle, cpFloat stiffness, cpFloat damping);
cpFloat                        cpDampedRotarySpringGetRestAngle        ( const cpConstraint *constraint);
void                           cpDampedRotarySpringSetRestAngle        ( cpConstraint *constraint, cpFloat restAngle);
cpFloat                        cpDampedRotarySpringGetStiffness        ( const cpConstraint *constraint);
void                           cpDampedRotarySpringSetStiffness        ( cpConstraint *constraint, cpFloat stiffness);
cpFloat                        cpDampedRotarySpringGetDamping          ( const cpConstraint *constraint);
void                           cpDampedRotarySpringSetDamping          ( cpConstraint *constraint, cpFloat damping);
cpDampedRotarySpringTorqueFunc cpDampedRotarySpringGetSpringTorqueFunc ( const cpConstraint *constraint);
void                           cpDampedRotarySpringSetSpringTorqueFunc ( cpConstraint *constraint, cpDampedRotarySpringTorqueFunc springTorqueFunc);
]])
--====================================================================
--#include "cpRotaryLimitJoint.h" from cpConstraint.h
ffi.cdef([[
cpBool              cpConstraintIsRotaryLimitJoint ( const cpConstraint *constraint);
cpRotaryLimitJoint* cpRotaryLimitJointAlloc        ( void);
cpRotaryLimitJoint* cpRotaryLimitJointInit         ( cpRotaryLimitJoint *joint, cpBody *a, cpBody *b, cpFloat min, cpFloat max);
cpConstraint*       cpRotaryLimitJointNew          ( cpBody *a, cpBody *b, cpFloat min, cpFloat max);
cpFloat             cpRotaryLimitJointGetMin       ( const cpConstraint *constraint);
void                cpRotaryLimitJointSetMin       ( cpConstraint *constraint, cpFloat min);
cpFloat             cpRotaryLimitJointGetMax       ( const cpConstraint *constraint);
void                cpRotaryLimitJointSetMax       ( cpConstraint *constraint, cpFloat max);
]])
--====================================================================
--#include "cpRatchetJoint.h" from cpConstraint.h
ffi.cdef([[
cpBool          cpConstraintIsRatchetJoint ( const cpConstraint *constraint);
cpRatchetJoint* cpRatchetJointAlloc        ( void);
cpRatchetJoint* cpRatchetJointInit         ( cpRatchetJoint *joint, cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet);
cpConstraint*   cpRatchetJointNew          ( cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet);
cpFloat         cpRatchetJointGetAngle     ( const cpConstraint *constraint);
void            cpRatchetJointSetAngle     ( cpConstraint *constraint, cpFloat angle);
cpFloat         cpRatchetJointGetPhase     ( const cpConstraint *constraint);
void            cpRatchetJointSetPhase     ( cpConstraint *constraint, cpFloat phase);
cpFloat         cpRatchetJointGetRatchet   ( const cpConstraint *constraint);
void            cpRatchetJointSetRatchet   ( cpConstraint *constraint, cpFloat ratchet);
]])
--====================================================================
--#include "cpGearJoint.h" from cpConstraint.h
ffi.cdef([[
cpBool        cpConstraintIsGearJoint      ( const cpConstraint *constraint);
cpGearJoint*  cpGearJointAlloc             ( void);
cpGearJoint*  cpGearJointInit              ( cpGearJoint *joint, cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio);
cpConstraint* cpGearJointNew               ( cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio);
cpFloat       cpGearJointGetPhase          ( const cpConstraint *constraint);
void          cpGearJointSetPhase          ( cpConstraint *constraint, cpFloat phase);
cpFloat       cpGearJointGetRatio          ( const cpConstraint *constraint);
void          cpGearJointSetRatio          ( cpConstraint *constraint, cpFloat ratio);
]])
--====================================================================
--#include "cpSimpleMotor.h" from cpConstraint.h
ffi.cdef([[
typedef struct cpSimpleMotor cpSimpleMotor;

cpBool         cpConstraintIsSimpleMotor   ( const cpConstraint *constraint);
cpSimpleMotor* cpSimpleMotorAlloc          ( void);
cpSimpleMotor* cpSimpleMotorInit           ( cpSimpleMotor *joint, cpBody *a, cpBody *b, cpFloat rate);
cpConstraint*  cpSimpleMotorNew            ( cpBody *a, cpBody *b, cpFloat rate);
cpFloat        cpSimpleMotorGetRate        ( const cpConstraint *constraint);
void           cpSimpleMotorSetRate        ( cpConstraint *constraint, cpFloat rate);
]])
--====================================================================
-- cpSpace.h
ffi.cdef([[
    
//MARK: Definitions
typedef cpBool (*cpCollisionBeginFunc     )( cpArbiter *arb, cpSpace *space, cpDataPointer userData);
typedef cpBool (*cpCollisionPreSolveFunc  )( cpArbiter *arb, cpSpace *space, cpDataPointer userData);
typedef void   (*cpCollisionPostSolveFunc )( cpArbiter *arb, cpSpace *space, cpDataPointer userData);
typedef void   (*cpCollisionSeparateFunc  )( cpArbiter *arb, cpSpace *space, cpDataPointer userData);

struct cpCollisionHandler {

    cpCollisionType          typeA; //const cpCollisionType typeA;
    cpCollisionType          typeB;//const cpCollisionType typeB;
    cpCollisionBeginFunc     beginFunc;
    cpCollisionPreSolveFunc  preSolveFunc;
    cpCollisionPostSolveFunc postSolveFunc;
    cpCollisionSeparateFunc  separateFunc;
    cpDataPointer userData;
};

//MARK: Memory and Initialization
cpSpace*      cpSpaceAlloc                  ( void);
cpSpace*      cpSpaceInit                   ( cpSpace *space);
cpSpace*      cpSpaceNew                    ( void);
void          cpSpaceDestroy                ( cpSpace *space);
void          cpSpaceFree                   ( cpSpace *space);

//MARK: Properties
int           cpSpaceGetIterations          ( const cpSpace *space);
void          cpSpaceSetIterations          ( cpSpace *space, int iterations);
cpVect        cpSpaceGetGravity             ( const cpSpace *space);
void          cpSpaceSetGravity             ( cpSpace *space, cpVect gravity);
cpFloat       cpSpaceGetDamping             ( const cpSpace *space);
void          cpSpaceSetDamping             ( cpSpace *space, cpFloat damping);
cpFloat       cpSpaceGetIdleSpeedThreshold  ( const cpSpace *space);
void          cpSpaceSetIdleSpeedThreshold  ( cpSpace *space, cpFloat idleSpeedThreshold);
cpFloat       cpSpaceGetSleepTimeThreshold  ( const cpSpace *space);
void          cpSpaceSetSleepTimeThreshold  ( cpSpace *space, cpFloat sleepTimeThreshold);
cpFloat       cpSpaceGetCollisionSlop       ( const cpSpace *space);
void          cpSpaceSetCollisionSlop       ( cpSpace *space, cpFloat collisionSlop);
cpFloat       cpSpaceGetCollisionBias       ( const cpSpace *space);
void          cpSpaceSetCollisionBias       ( cpSpace *space, cpFloat collisionBias);
cpTimestamp   cpSpaceGetCollisionPersistence( const cpSpace *space);
void          cpSpaceSetCollisionPersistence( cpSpace *space, cpTimestamp collisionPersistence);
cpDataPointer cpSpaceGetUserData            ( const cpSpace *space);
void          cpSpaceSetUserData            ( cpSpace *space, cpDataPointer userData);
cpBody*       cpSpaceGetStaticBody          ( const cpSpace *space);
cpFloat       cpSpaceGetCurrentTimeStep     ( const cpSpace *space);
cpBool        cpSpaceIsLocked               ( cpSpace *space);

//MARK: Collision Handlers
cpCollisionHandler* cpSpaceAddDefaultCollisionHandler( cpSpace *space);
cpCollisionHandler* cpSpaceAddCollisionHandler       ( cpSpace *space, cpCollisionType a, cpCollisionType b);
cpCollisionHandler* cpSpaceAddWildcardHandler        ( cpSpace *space, cpCollisionType type);

//MARK: Add/Remove objects
cpShape*      cpSpaceAddShape                ( cpSpace *space, cpShape *shape);
cpBody*       cpSpaceAddBody                 ( cpSpace *space, cpBody *body);
cpConstraint* cpSpaceAddConstraint           ( cpSpace *space, cpConstraint *constraint);
void          cpSpaceRemoveShape             ( cpSpace *space, cpShape *shape);
void          cpSpaceRemoveBody              ( cpSpace *space, cpBody *body);
void          cpSpaceRemoveConstraint        ( cpSpace *space, cpConstraint *constraint);
cpBool        cpSpaceContainsShape           ( cpSpace *space, cpShape *shape);
cpBool        cpSpaceContainsBody            ( cpSpace *space, cpBody *body);
cpBool        cpSpaceContainsConstraint      ( cpSpace *space, cpConstraint *constraint);

//MARK: Post-Step Callbacks
typedef void (*cpPostStepFunc)               ( cpSpace *space, void *key, void *data);
typedef void (*cpSpacePointQueryFunc)        ( cpShape *shape, cpVect point, cpFloat distance, cpVect gradient, void *data);
typedef void (*cpSpaceSegmentQueryFunc)      ( cpShape *shape, cpVect point, cpVect normal, cpFloat alpha, void *data);
typedef void (*cpSpaceBBQueryFunc)           ( cpShape *shape, void *data);
typedef void (*cpSpaceShapeQueryFunc)        ( cpShape *shape, cpContactPointSet *points, void *data);
typedef void (*cpSpaceBodyIteratorFunc)      ( cpBody *body, void *data);
typedef void (*cpSpaceShapeIteratorFunc)     ( cpShape *shape, void *data);
typedef void (*cpSpaceConstraintIteratorFunc)( cpConstraint *constraint, void *data);

cpBool       cpSpaceAddPostStepCallback      ( cpSpace *space, cpPostStepFunc func, void *key, void *data);
void         cpSpacePointQuery               ( cpSpace *space, cpVect point, cpFloat maxDistance, cpShapeFilter filter, cpSpacePointQueryFunc func, void *data);
cpShape*     cpSpacePointQueryNearest        ( cpSpace *space, cpVect point, cpFloat maxDistance, cpShapeFilter  filter, cpPointQueryInfo *out);
void         cpSpaceSegmentQuery             ( cpSpace *space, cpVect start, cpVect end, cpFloat radius, cpShapeFilter filter, cpSpaceSegmentQueryFunc func, void *data);
cpShape*     cpSpaceSegmentQueryFirst        ( cpSpace *space, cpVect start, cpVect end, cpFloat radius, cpShapeFilter filter, cpSegmentQueryInfo *out);
void         cpSpaceBBQuery                  ( cpSpace *space, cpBB bb, cpShapeFilter filter, cpSpaceBBQueryFunc func, void *data);
cpBool       cpSpaceShapeQuery               ( cpSpace *space, cpShape *shape, cpSpaceShapeQueryFunc func, void  *data);
void         cpSpaceEachBody                 ( cpSpace *space, cpSpaceBodyIteratorFunc func , void *data);
void         cpSpaceEachShape                ( cpSpace *space, cpSpaceShapeIteratorFunc func, void *data);
void         cpSpaceEachConstraint           ( cpSpace *space, cpSpaceConstraintIteratorFunc func, void *data);

//MARK: Indexing
void cpSpaceReindexStatic                    ( cpSpace *space);
void cpSpaceReindexShape                     ( cpSpace *space, cpShape *shape);
void cpSpaceReindexShapesForBody             ( cpSpace *space, cpBody *body);
void cpSpaceUseSpatialHash                   ( cpSpace *space, cpFloat dim, int count);

//MARK: Time Stepping
void cpSpaceStep(cpSpace *space, cpFloat dt);

//====================================================================
// #ifndef CP_SPACE_DISABLE_DEBUG_API
//====================================================================
//MARK: Debug API
typedef struct cpSpaceDebugColor {
    float r, g, b, a;
} cpSpaceDebugColor;

typedef void              (*cpSpaceDebugDrawCircleImpl        )( cpVect pos, cpFloat angle, cpFloat radius, cpSpaceDebugColor outlineColor, cpSpaceDebugColor fillColor, cpDataPointer data);
typedef void              (*cpSpaceDebugDrawSegmentImpl       )( cpVect a, cpVect b, cpSpaceDebugColor color, cpDataPointer data);
typedef void              (*cpSpaceDebugDrawFatSegmentImpl    )( cpVect a, cpVect b, cpFloat radius, cpSpaceDebugColor outlineColor, cpSpaceDebugColor fillColor, cpDataPointer data);
typedef void              (*cpSpaceDebugDrawPolygonImpl       )( int count, const cpVect *verts, cpFloat radius, cpSpaceDebugColor outlineColor, cpSpaceDebugColor fillColor, cpDataPointer data);
typedef void              (*cpSpaceDebugDrawDotImpl           )( cpFloat size, cpVect pos, cpSpaceDebugColor color, cpDataPointer data);
typedef cpSpaceDebugColor (*cpSpaceDebugDrawColorForShapeImpl )( cpShape *shape, cpDataPointer data);

typedef enum cpSpaceDebugDrawFlags {
    CP_SPACE_DEBUG_DRAW_SHAPES = 1<<0,
    CP_SPACE_DEBUG_DRAW_CONSTRAINTS = 1<<1,
    CP_SPACE_DEBUG_DRAW_COLLISION_POINTS = 1<<2,
} cpSpaceDebugDrawFlags;

typedef struct cpSpaceDebugDrawOptions {
    cpSpaceDebugDrawCircleImpl        drawCircle;
    cpSpaceDebugDrawSegmentImpl       drawSegment;
    cpSpaceDebugDrawFatSegmentImpl    drawFatSegment;
    cpSpaceDebugDrawPolygonImpl       drawPolygon;
    cpSpaceDebugDrawDotImpl           drawDot;
    cpSpaceDebugDrawFlags             flags;
    cpSpaceDebugColor                 shapeOutlineColor;
    cpSpaceDebugDrawColorForShapeImpl colorForShape;
    cpSpaceDebugColor                 constraintColor;
    cpSpaceDebugColor                 collisionPointColor;
    cpDataPointer                     data;
} cpSpaceDebugDrawOptions;

void cpSpaceDebugDraw (cpSpace *space, cpSpaceDebugDrawOptions *options);
//====================================================================
//#endif
//====================================================================
]])
--====================================================================
-- chipmunk_unsafe.h
ffi.cdef([[
void cpCircleShapeSetRadius    ( cpShape *shape, cpFloat radius);
void cpCircleShapeSetOffset    ( cpShape *shape, cpVect offset);
void cpSegmentShapeSetEndpoints( cpShape *shape, cpVect a, cpVect b);
void cpSegmentShapeSetRadius   ( cpShape *shape, cpFloat radius);
void cpPolyShapeSetVerts       ( cpShape *shape, int count, cpVect *verts, cpTransform transform);
void cpPolyShapeSetVertsRaw    ( cpShape *shape, int count, cpVect *verts);
void cpPolyShapeSetRadius      ( cpShape *shape, cpFloat radius);
]])
--====================================================================
-- cpHastySpace.h
ffi.cdef([[
struct cpHastySpace;
typedef struct cpHastySpace cpHastySpace;
cpSpace*       cpHastySpaceNew        ( void);
void           cpHastySpaceFree       ( cpSpace *space);
void           cpHastySpaceSetThreads ( cpSpace *space, unsigned long threads);
unsigned long  cpHastySpaceGetThreads ( cpSpace *space);
void           cpHastySpaceStep       ( cpSpace *space, cpFloat dt);
]])
--====================================================================
-- cpMarch.h
ffi.cdef([[
typedef cpFloat (*cpMarchSampleFunc  )( cpVect point, void *data);
typedef void    (*cpMarchSegmentFunc )( cpVect v0, cpVect v1, void *data);

void cpMarchSoft(
  cpBB bb, unsigned long x_samples, unsigned long y_samples, cpFloat threshold,
  cpMarchSegmentFunc segment, void *segment_data,
  cpMarchSampleFunc  sample , void *sample_data
);

void cpMarchHard(
  cpBB bb, unsigned long x_samples, unsigned long y_samples, cpFloat threshold,
  cpMarchSegmentFunc segment, void *segment_data,
  cpMarchSampleFunc sample, void *sample_data
);
]])
--====================================================================
-- cpPolyline.h
ffi.cdef([[
typedef struct cpPolyline {
  int count, capacity;
  cpVect verts[];
} cpPolyline;

void              cpPolylineFree             ( cpPolyline *line );
cpBool            cpPolylineIsClosed         ( cpPolyline *line );
cpPolyline       *cpPolylineSimplifyCurves   ( cpPolyline *line, cpFloat tol);
cpPolyline       *cpPolylineSimplifyVertexes ( cpPolyline *line, cpFloat tol);
cpPolyline       *cpPolylineToConvexHull     ( cpPolyline *line, cpFloat tol);

/// Polyline sets are collections of polylines, generally built by cpMarchSoft() or cpMarchHard().
typedef struct cpPolylineSet {
  int count, capacity;
  cpPolyline **lines;
} cpPolylineSet;

cpPolylineSet*    cpPolylineSetAlloc           ( void);
cpPolylineSet*    cpPolylineSetInit            ( cpPolylineSet *set);
cpPolylineSet*    cpPolylineSetNew             ( void);
void              cpPolylineSetDestroy         ( cpPolylineSet *set, cpBool freePolylines);
void              cpPolylineSetFree            ( cpPolylineSet *set, cpBool freePolylines);
void              cpPolylineSetCollectSegment  ( cpVect v0, cpVect v1, cpPolylineSet *lines);
cpPolylineSet*    cpPolylineConvexDecomposition( cpPolyline *line, cpFloat tol);

//#define cpPolylineConvexDecomposition_BETA cpPolylineConvexDecomposition
]])
--====================================================================
-- cpRobust.h
ffi.cdef([[
//private
//cpBool cpCheckSignedArea(const cpVect a, const cpVect b, const cpVect c);
]])
--====================================================================
-- chipmunk_ffi.h
ffi.cdef([[
cpVect  (*_cpv)                        ( const cpFloat x, const cpFloat y);
cpBool  (*_cpveql)                     ( const cpVect v1, const cpVect v2);
cpVect  (*_cpvadd)                     ( const cpVect v1, const cpVect v2);
cpVect  (*_cpvsub)                     ( const cpVect v1, const cpVect v2);
cpVect  (*_cpvneg)                     ( const cpVect v);
cpVect  (*_cpvmult)                    ( const cpVect v, const cpFloat s);
cpFloat (*_cpvdot)                     ( const cpVect v1, const cpVect v2);
cpFloat (*_cpvcross)                   ( const cpVect v1, const cpVect v2);
cpVect  (*_cpvperp)                    ( const cpVect v){return cpvperp(v);};
cpVect  (*_cpvrperp)                   ( const cpVect v){return cpvrperp(v);};
cpVect  (*_cpvproject)                 ( const cpVect v1, const cpVect v2);
cpVect  (*_cpvforangle)                ( const cpFloat a);
cpFloat (*_cpvtoangle)                 ( const cpVect v);
cpVect  (*_cpvrotate)                  ( const cpVect v1, const cpVect v2);
cpVect  (*_cpvunrotate)                ( const cpVect v1, const cpVect v2);
cpFloat (*_cpvlengthsq)                ( const cpVect v);
cpFloat (*_cpvlength)                  ( const cpVect v);
cpVect  (*_cpvlerp)                    ( const cpVect v1, const cpVect v2, const cpFloat t);
cpVect  (*_cpvnormalize)               ( const cpVect v);
cpVect  (*_cpvslerp)                   ( const cpVect v1, const cpVect v2, const cpFloat t);
cpVect  (*_cpvslerpconst)              ( const cpVect v1, const cpVect v2, const cpFloat a);
cpVect  (*_cpvclamp)                   ( const cpVect v, const cpFloat len);
cpVect  (*_cpvlerpconst)               ( cpVect v1, cpVect v2, cpFloat d);
cpFloat (*_cpvdist)                    ( const cpVect v1, const cpVect v2);
cpFloat (*_cpvdistsq)                  ( const cpVect v1, const cpVect v2);
cpBool  (*_cpvnear)                    ( const cpVect v1, const cpVect v2, const cpFloat dist);
//====================================================================
cpFloat (*_cpfmax)                     ( cpFloat a, cpFloat b) { return cpfmax ( a, b) ;};
cpFloat (*_cpfmin)                     ( cpFloat a, cpFloat b) { return cpfmin ( a, b) ;};
cpFloat (*_cpfabs)                     ( cpFloat f) { return cpfabs ( f) ;};
cpFloat (*_cpfclamp)                   ( cpFloat f, cpFloat min, cpFloat max) { return cpfclamp ( f,min,max) ;};
cpFloat (*_cpflerp)                    ( cpFloat f1, cpFloat f2, cpFloat t) { return cpflerp (f1,f2,t) ;};
cpFloat (*_cpflerpconst)               ( cpFloat f1, cpFloat f2, cpFloat d) { return cpflerpconst ( f1,f2,d) ;};
//====================================================================
cpBB    (*_cpBBNew)                    ( const cpFloat l, const cpFloat b, const cpFloat r, const cpFloat t) {return  cpBBNew ( l, b, r,  t) ;};
cpBB    (*_cpBBNewForExtents)          ( const cpVect c, const cpFloat hw, const cpFloat hh){return  cpBBNewForExtents ( c,hw,hh) ;};
cpBB    (*_cpBBNewForCircle)           ( const cpVect p, const cpFloat r){return  cpBBNewForCircle ( p,r) ;};
cpBool  (*_cpBBIntersects)             ( const cpBB a, const cpBB b){return  cpBBIntersects (a,b) ;};
cpBool  (*_cpBBContainsBB)             ( const cpBB bb, const cpBB other){return  cpBBContainsBB (bb,other) ;};
cpBool  (*_cpBBContainsVect)           ( const cpBB bb, const cpVect v){return  cpBBContainsVect (bb,v) ;};
cpBB    (*_cpBBMerge)                  ( const cpBB a, const cpBB b){return  cpBBMerge (a,b) ;};
cpBB    (*_cpBBExpand)                 ( const cpBB bb, const cpVect v){return  cpBBExpand (bb,v) ;};
cpVect  (*_cpBBCenter)                 ( cpBB bb){return  cpBBCenter(bb) ;};
cpFloat (*_cpBBArea)                   ( cpBB bb){return  cpBBArea (bb) ;};
cpFloat (*_cpBBMergedArea)             ( cpBB a, cpBB b){return  cpBBMergedArea (a,b) ;};
cpFloat (*_cpBBSegmentQuery)           ( cpBB bb, cpVect a, cpVect b){return  cpBBSegmentQuery (bb,a,b) ;};
cpBool  (*_cpBBIntersectsSegment)      ( cpBB bb, cpVect a, cpVect b){return  cpBBIntersectsSegment (bb,a,b) ;};
cpVect  (*_cpBBClampVect)              ( const cpBB bb, const cpVect v){return  cpBBClampVect (bb,v) ;};
cpVect  (*_cpBBWrapVect)               ( const cpBB bb, const cpVect v){return  cpBBWrapVect (bb,v) ;};
cpBB    (*_cpBBOffset)                 ( const cpBB bb, const cpVect v){return  cpBBOffset (bb,v) ;};
//====================================================================
void    (*_cpSpatialIndexDestroy)      ( cpSpatialIndex *index) {return cpSpatialIndexDestroy(index);};
int     (*_cpSpatialIndexCount)        ( cpSpatialIndex *index) {return _cpSpatialIndexCount(index);};
void    (*_cpSpatialIndexEach)         ( cpSpatialIndex *index, cpSpatialIndexIteratorFunc func, void *data) {return cpSpatialIndexEach(index,func,data);};
cpBool  (*_cpSpatialIndexContains)     ( cpSpatialIndex *index, void *obj, cpHashValue hashid) {return cpSpatialIndexContains(index,obj,hashid);};
void    (*_cpSpatialIndexInsert)       ( cpSpatialIndex *index, void *obj, cpHashValue hashid) {return cpSpatialIndexInsert(index,obj,hashid);};
void    (*_cpSpatialIndexRemove)       ( cpSpatialIndex *index, void *obj, cpHashValue hashid) {return cpSpatialIndexRemove(index,obj,hashid);};
void    (*_cpSpatialIndexReindex)      ( cpSpatialIndex *index) {return cpSpatialIndexReindex(index);};
void    (*_cpSpatialIndexReindexObject)( cpSpatialIndex *index, void *obj, cpHashValue hashid) {return cpSpatialIndexReindexObject(index,obj,hashid);};
void    (*_cpSpatialIndexQuery)        ( cpSpatialIndex *index
                                       ,void *obj
                                       ,cpBB bb
                                       ,cpSpatialIndexQueryFunc func
                                       ,void *data){return cpSpatialIndexQuery(index,obj,bb,func,data);};
void    (*_cpSpatialIndexSegmentQuery) ( cpSpatialIndex *index
                                       ,void *obj
                                       ,cpVect a
                                       ,cpVect b
                                       ,cpFloat t_exit
                                       ,cpSpatialIndexSegmentQueryFunc func
                                       ,void *data){return cpSpatialIndexSegmentQuery(index,obj,a,b,t_exit,func,data);};
void    (*_cpSpatialIndexReindexQuery) ( cpSpatialIndex *index, cpSpatialIndexQueryFunc func, void *data) {return cpSpatialIndexReindexQuery(index,func,data);};

extern  const char *cpVersionString;
cpFloat cpMomentForCircle   ( cpFloat m, cpFloat r1, cpFloat r2, cpVect offset);
cpFloat cpAreaForCircle     ( cpFloat r1, cpFloat r2);
cpFloat cpMomentForSegment  ( cpFloat m, cpVect a, cpVect b, cpFloat radius);
cpFloat cpAreaForSegment    ( cpVect a, cpVect b, cpFloat radius);
cpFloat cpMomentForPoly     ( cpFloat m, int count, const cpVect *verts, cpVect offset, cpFloat radius);
cpFloat cpAreaForPoly       ( const int count, const cpVect *verts, cpFloat radius);
cpVect  cpCentroidForPoly   ( const int count, const cpVect *verts);
cpFloat cpMomentForBox      ( cpFloat m, cpFloat width, cpFloat height);
cpFloat cpMomentForBox2     ( cpFloat m, cpBB box);
int     cpConvexHull        ( int count, const cpVect *verts, cpVect *result, int *first, cpFloat tol);
]])
--====================================================================
-- cpPrivate.h
ffi.cdef([[

struct cpArray {
    int num, max;
    void **arr;
};

struct cpBody {
    cpBodyVelocityFunc velocity_func;// Integration functions
    cpBodyPositionFunc position_func;
    cpFloat m;// mass and it's inverse
    cpFloat m_inv;
    cpFloat i; // moment of inertia and it's inverse
    cpFloat i_inv;
    cpVect cog; // center of gravity
    cpVect p; // position, velocity, force
    cpVect v;
    cpVect f;
    cpFloat a;// Angle, angular velocity, torque (radians)
    cpFloat w;
    cpFloat t;
    cpTransform transform;
    cpDataPointer userData;
    cpVect v_bias;// "pseudo-velocities" used for eliminating overlap.
    cpFloat w_bias; // Erin Catto has some papers that talk about what these are.
    cpSpace *space;
    cpShape *shapeList;
    cpArbiter *arbiterList;
    cpConstraint *constraintList;
    struct {
        cpBody *root;
        cpBody *next;
        cpFloat idleTime;
    } sleeping;
};

typedef struct cpArbiter cpArbiter;
typedef struct cpContact cpContact;

struct cpArbiterThread {
    struct cpArbiter *next, *prev;
};

enum cpArbiterState {
    CP_ARBITER_STATE_FIRST_COLLISION,
    CP_ARBITER_STATE_NORMAL,
    CP_ARBITER_STATE_IGNORE,
    CP_ARBITER_STATE_CACHED,
    CP_ARBITER_STATE_INVALIDATED,
};

struct cpContact {
    cpVect r1, r2;
    cpFloat nMass, tMass;
    cpFloat bounce;
    cpFloat jnAcc, jtAcc, jBias;
    cpFloat bias;
    cpHashValue hash;
};

struct cpCollisionInfo {
    const cpShape *a, *b;
    cpCollisionID id;
    cpVect n;
    int count;
    struct cpContact *arr;
};

struct cpArbiter {
    cpFloat e;
    cpFloat u;
    cpVect surface_vr;
    cpDataPointer data;
    const cpShape *a, *b;
    cpBody *body_a, *body_b;
    struct cpArbiterThread thread_a, thread_b;
    int count;
    struct cpContact *contacts;
    cpVect n;
    cpCollisionHandler *handler, *handlerA, *handlerB;
    cpBool swapped;
    cpTimestamp stamp;
    enum cpArbiterState state;
};

struct cpShapeMassInfo {
    cpFloat m;
    cpFloat i;
    cpVect cog;
    cpFloat area;
};

typedef enum cpShapeType{
    CP_CIRCLE_SHAPE,
    CP_SEGMENT_SHAPE,
    CP_POLY_SHAPE,
    CP_NUM_SHAPES
} cpShapeType;

typedef struct cpShapeClass cpShapeClass;
typedef cpBB (*cpShapeCacheDataImpl    )( cpShape *shape, cpTransform transform);
typedef void (*cpShapeDestroyImpl      )( cpShape *shape);
typedef void (*cpShapePointQueryImpl   )( const cpShape *shape, cpVect p, cpPointQueryInfo *info);
typedef void (*cpShapeSegmentQueryImpl )( const cpShape *shape, cpVect a, cpVect b, cpFloat radius, cpSegmentQueryInfo *info);

struct cpShapeClass {
    cpShapeType type;
    cpShapeCacheDataImpl cacheData;
    cpShapeDestroyImpl destroy;
    cpShapePointQueryImpl pointQuery;
    cpShapeSegmentQueryImpl segmentQuery;
};

struct cpShape {
    const cpShapeClass *klass;
    cpSpace *space;
    cpBody *body;
    struct cpShapeMassInfo massInfo;
    cpBB bb;
    cpBool sensor;
    cpFloat e;
    cpFloat u;
    cpVect surfaceV;
    cpDataPointer userData;
    cpCollisionType type;
    cpShapeFilter filter;
    cpShape *next;
    cpShape *prev;
    cpHashValue hashid;
};

struct cpCircleShape {
    cpShape shape;
    cpVect c, tc;
    cpFloat r;
};

struct cpSegmentShape {
    cpShape shape;
    cpVect a, b, n;
    cpVect ta, tb, tn;
    cpFloat r;
    cpVect a_tangent, b_tangent;
};

struct cpSplittingPlane {
    cpVect v0, n;
};

enum
{
CP_POLY_SHAPE_INLINE_ALLOC=6
};

struct cpPolyShape {
    cpShape shape;
    cpFloat r;
    int count;
    struct cpSplittingPlane *planes;
    struct cpSplittingPlane _planes[2*CP_POLY_SHAPE_INLINE_ALLOC];
};

typedef void    (*cpConstraintPreStepImpl            )( cpConstraint *constraint, cpFloat dt);
typedef void    (*cpConstraintApplyCachedImpulseImpl )( cpConstraint *constraint, cpFloat dt_coef);
typedef void    (*cpConstraintApplyImpulseImpl       )( cpConstraint *constraint, cpFloat dt);
typedef cpFloat (*cpConstraintGetImpulseImpl         )( cpConstraint *constraint );

typedef struct cpConstraintClass {
    cpConstraintPreStepImpl preStep;
    cpConstraintApplyCachedImpulseImpl applyCachedImpulse;
    cpConstraintApplyImpulseImpl applyImpulse;
    cpConstraintGetImpulseImpl getImpulse;
} cpConstraintClass;

struct cpConstraint {
    const cpConstraintClass *klass;
    cpSpace *space;
    cpBody *a, *b;
    cpConstraint *next_a, *next_b;
    cpFloat maxForce;
    cpFloat errorBias;
    cpFloat maxBias;
    cpBool collideBodies;
    cpConstraintPreSolveFunc preSolve;
    cpConstraintPostSolveFunc postSolve;
    cpDataPointer userData;
};

struct cpPinJoint {
    cpConstraint constraint;
    cpVect anchorA, anchorB;
    cpFloat dist;
    cpVect r1, r2;
    cpVect n;
    cpFloat nMass;
    cpFloat jnAcc;
    cpFloat bias;
};

struct cpSlideJoint {
    cpConstraint constraint;
    cpVect anchorA, anchorB;
    cpFloat min, max;
    cpVect r1, r2;
    cpVect n;
    cpFloat nMass;
    cpFloat jnAcc;
    cpFloat bias;
};

struct cpPivotJoint {
    cpConstraint constraint;
    cpVect anchorA, anchorB;
    cpVect r1, r2;
    cpMat2x2 k;
    cpVect jAcc;
    cpVect bias;
};

struct cpGrooveJoint {
    cpConstraint constraint;
    cpVect grv_n, grv_a, grv_b;
    cpVect  anchorB;
    cpVect grv_tn;
    cpFloat clamp;
    cpVect r1, r2;
    cpMat2x2 k;
    cpVect jAcc;
    cpVect bias;
};

struct cpDampedSpring {
    cpConstraint constraint;
    cpVect anchorA, anchorB;
    cpFloat restLength;
    cpFloat stiffness;
    cpFloat damping;
    cpDampedSpringForceFunc springForceFunc;
    cpFloat target_vrn;
    cpFloat v_coef;
    cpVect r1, r2;
    cpFloat nMass;
    cpVect n;
    cpFloat jAcc;
};

struct cpDampedRotarySpring {
    cpConstraint constraint;
    cpFloat restAngle;
    cpFloat stiffness;
    cpFloat damping;
    cpDampedRotarySpringTorqueFunc springTorqueFunc;
    cpFloat target_wrn;
    cpFloat w_coef;
    cpFloat iSum;
    cpFloat jAcc;
};

struct cpRotaryLimitJoint {
    cpConstraint constraint;
    cpFloat min, max;
    cpFloat iSum;
    cpFloat bias;
    cpFloat jAcc;
};

struct cpRatchetJoint {
    cpConstraint constraint;
    cpFloat angle, phase, ratchet;
    cpFloat iSum;
    cpFloat bias;
    cpFloat jAcc;
};

struct cpGearJoint {
    cpConstraint constraint;
    cpFloat phase, ratio;
    cpFloat ratio_inv;
    cpFloat iSum;
    cpFloat bias;
    cpFloat jAcc;
};

struct cpSimpleMotor {
    cpConstraint constraint;
    cpFloat rate;
    cpFloat iSum;
    cpFloat jAcc;
};

typedef struct  cpContactBufferHeader cpContactBufferHeader;
typedef void ( *cpSpaceArbiterApplyImpulseFunc ) (cpArbiter *arb);

struct cpSpace {
    int iterations;
    cpVect gravity;
    cpFloat damping;
    cpFloat idleSpeedThreshold;
    cpFloat sleepTimeThreshold;
    cpFloat collisionSlop;
    cpFloat collisionBias;
    cpTimestamp collisionPersistence;
    cpDataPointer userData;
    cpTimestamp stamp;
    cpFloat curr_dt;
    cpArray *dynamicBodies;
    cpArray *staticBodies;
    cpArray *rousedBodies;
    cpArray *sleepingComponents;
    cpHashValue shapeIDCounter;
    cpSpatialIndex *staticShapes;
    cpSpatialIndex *dynamicShapes;
    cpArray *constraints;
    cpArray *arbiters;
    cpContactBufferHeader *contactBuffersHead;
    cpHashSet *cachedArbiters;
    cpArray *pooledArbiters;
    cpArray *allocatedBuffers;
    unsigned int locked;
    cpBool usesWildcards;
    cpHashSet *collisionHandlers;
    cpCollisionHandler defaultHandler;
    cpBool skipPostStep;
    cpArray *postStepCallbacks;
    cpBody *staticBody;
    cpBody _staticBody;
};

typedef struct cpPostStepCallback {
    cpPostStepFunc func;
    void *key;
    void *data;
} cpPostStepCallback;

]])
--====================================================================

--====================================================================
local cp =ffi.load( TargetDllName )
--====================================================================
return cp;
--====================================================================
