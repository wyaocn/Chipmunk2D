#include <lua.h>
#include <lauxlib.h>

#include "chipmunk/chipmunk.h"

static int
_space_new(lua_State *L) {
    cpSpace *space = cpSpaceNew();
    lua_getfield(L, LUA_REGISTRYINDEX, "chipmunk_collision_data");
    lua_newtable(L);
    lua_rawsetp(L, -2, space);
    lua_getfield(L, LUA_REGISTRYINDEX, "chipmunk_collision_handler");
    lua_newtable(L);
    lua_rawsetp(L, -2, space);
    lua_pushlightuserdata(L, space);
    return 1;
}

typedef struct {
    lua_State *L;
    cpSpace *space;
} CollisionUserData;

static cpBool
_collision_handler(cpArbiter *arb, cpSpace *space, cpDataPointer userData) {
	CP_ARBITER_GET_BODIES(arb, body1, body2);
    CollisionUserData *data = (CollisionUserData *)userData;
    lua_State *L = data->L;
    lua_settop(L, 0);
    lua_getfield(L, LUA_REGISTRYINDEX, "chipmunk_collision_handler");
    lua_rawgetp(L, -1, data->space);
    lua_rawgetp(L, -1, data);
    lua_pushlightuserdata(L, body1);
    lua_pushlightuserdata(L, body2);
    int err = lua_pcall(L, 2, 1, 0);
    if (err != LUA_OK) {
        cpMessage("collision handler failed", __FILE__, __LINE__, 1, 0, "collision handler err:%s", lua_tostring(L,-1));
	    lua_pop(L,3);
        return cpTrue;
    }
    cpBool val = (cpBool)lua_toboolean(L, -1);
    lua_pop(L, 3);
    return val;
}

static int
_space_add_collision_handler(lua_State *L) {
    cpSpace *space = (cpSpace *)lua_touserdata(L, 1);
    if (space == NULL)
		return luaL_error(L, "Invalid param space");
    if (!lua_isfunction(L, 4))
		return luaL_error(L, "Invalid param func");
    cpCollisionType a = luaL_checknumber(L,2);
    cpCollisionType b = luaL_checknumber(L,3);
    lua_getfield(L, LUA_REGISTRYINDEX, "chipmunk_collision_data");
    lua_getfield(L, LUA_REGISTRYINDEX, "chipmunk_collision_handler");
    lua_rawgetp(L, -2, space);
    lua_rawgetp(L, -2, space);
    CollisionUserData *userData = (CollisionUserData *)lua_newuserdata(L, sizeof(CollisionUserData));
    userData->L = L;
    userData->space = space;
    lua_rawsetp(L, -3, userData);
    lua_pushvalue(L, 4);
    lua_rawsetp(L, -2, userData);
    cpCollisionHandler *handler = cpSpaceAddCollisionHandler(space, a, b);
    handler->beginFunc = (cpCollisionBeginFunc)_collision_handler;
    handler->userData = (cpDataPointer)userData;
    return 0;
}

static int
_space_add_body(lua_State *L) {
    cpSpace *space = (cpSpace *)lua_touserdata(L, 1);
    cpBody *body = (cpBody *)lua_touserdata(L, 2);
    if (space == NULL || body == NULL)
		return luaL_error(L, "Invalid param space or body");
    cpSpaceAddBody(space, body);
    return 0;
}

static int
_space_remove_body(lua_State *L) {
    cpSpace *space = (cpSpace *)lua_touserdata(L, 1);
    cpBody *body = (cpBody *)lua_touserdata(L, 2);
    if (space == NULL || body == NULL)
		return luaL_error(L, "Invalid param space or body");
    cpSpaceRemoveBody(space, body);
    return 0;
}

static int
_space_free(lua_State *L) {
    cpSpace *space = (cpSpace *)lua_touserdata(L, 1);
    if (space == NULL)
		return luaL_error(L, "Invalid param space");
    lua_getfield(L, LUA_REGISTRYINDEX, "chipmunk_collision_data");
    lua_pushnil(L);
    lua_rawsetp(L, -2, space);
    lua_getfield(L, LUA_REGISTRYINDEX, "chipmunk_collision_handler");
    lua_pushnil(L);
    lua_rawsetp(L, -2, space);
    cpSpaceFree(space);
    return 0;
}

static int
_space_step(lua_State *L) {
    cpSpace *space = (cpSpace *)lua_touserdata(L, 1);
    if (space == NULL)
		return luaL_error(L, "Invalid param space");
    cpFloat dt = (cpFloat)luaL_checknumber(L, 2);
    cpSpaceStep(space, dt);
    return 0;
}

static int
_body_new(lua_State *L) {
    cpFloat mass = (cpFloat)luaL_checknumber(L, 1);
    cpFloat moment = (cpFloat)luaL_checknumber(L, 2);
    cpBody *body = cpBodyNew(mass, moment);
    lua_pushlightuserdata(L, body);
    return 1;
}

static int
_body_new_kinematic(lua_State *L) {
    cpBody *body = cpBodyNewKinematic();
    lua_pushlightuserdata(L, body);
    return 1;
}

static int
_body_new_static(lua_State *L) {
    cpBody *body = cpBodyNewStatic();
    lua_pushlightuserdata(L, body);
    return 1;
}

static int
_body_local_to_world_vector(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpVect vect = cpv(luaL_checknumber(L,2),luaL_checknumber(L,3));
    vect = cpvrotate(vect, cpBodyGetRotation(body));
    lua_pushnumber(L, vect.x);
    lua_pushnumber(L, vect.y);
    return 2;
}

static int
_body_local_to_world_point(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpVect point = cpv(luaL_checknumber(L,2),luaL_checknumber(L,3));
    point = cpBodyLocalToWorld(body, point);
    lua_pushnumber(L, point.x);
    lua_pushnumber(L, point.y);
    return 2;
}

static int
_body_get_position(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpVect vect = cpBodyGetPosition(body);
    lua_pushnumber(L, vect.x);
    lua_pushnumber(L, vect.y);
    return 2;
}

static int
_body_set_position(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpVect vect = cpv(luaL_checknumber(L,2),luaL_checknumber(L,3));
    cpBodySetPosition(body, vect);
    return 0;
}

static int
_space_reindex_shapes_for_body(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpSpace *space = cpBodyGetSpace(body);
    if (space == NULL)
		return luaL_error(L, "body not in space yet");
    cpSpaceReindexShapesForBody(space, body);
    return 0;
}

static int
_body_get_forward(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpVect vect = cpvperp(cpBodyGetRotation(body));
    lua_pushnumber(L, vect.x);
    lua_pushnumber(L, vect.y);
    return 2;
}

static int
_body_get_angle(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpFloat angle = cpBodyGetAngle(body);
    lua_pushnumber(L, angle);
    return 1;
}

static int
_body_set_angle(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpFloat angle = luaL_checknumber(L,2);
    cpBodySetAngle(body, angle);
    return 0;
}

static int
_body_set_velocity(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpVect speed = cpv(luaL_checknumber(L,2),luaL_checknumber(L,3));
    cpBodySetVelocity(body, speed);
    return 0;
}

static int
_body_get_velocity(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpVect vect = cpBodyGetVelocity(body);
    lua_pushnumber(L, vect.x);
    lua_pushnumber(L, vect.y);
    return 2;
}

static int
_body_free(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpBodyFree(body);
    return 0;
}

static int
_poly_shape_new(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    if (!lua_istable(L, 2))
		return luaL_error(L, "Invalid param verts");
    if (!lua_istable(L, 3))
		return luaL_error(L, "Invalid param transform");
    int i, count = luaL_len(L, 2);
    cpVect verts[count];
    for(i=0;i<count;i++) {
        lua_geti(L, 2, i+1);
        lua_geti(L, -1, 1);
        lua_geti(L, -2, 2);
        verts[i] = cpv(luaL_checknumber(L,-2),luaL_checknumber(L,-1));
        lua_pop(L, 3);
    }
    lua_getfield(L, 3, "a");
    lua_getfield(L, 3, "b");
    lua_getfield(L, 3, "c");
    lua_getfield(L, 3, "d");
    lua_getfield(L, 3, "tx");
    lua_getfield(L, 3, "ty");
    cpTransform transform = cpTransformNew(
        luaL_optnumber(L, -6, 0),
        luaL_optnumber(L, -5, 0),
        luaL_optnumber(L, -4, 0),
        luaL_optnumber(L, -3, 0),
        luaL_optnumber(L, -2, 0),
        luaL_optnumber(L, -1, 0)
    );
    lua_pop(L, 6);
    cpFloat radius = luaL_optnumber(L, 4, 0);
    cpShape *shape = cpPolyShapeNew(body,count,verts,transform,radius);
    lua_pushlightuserdata(L, shape);
    return 1;
}

static int
_circle_shape_new(lua_State *L) {
    cpBody *body = (cpBody *)lua_touserdata(L, 1);
    if (body == NULL)
		return luaL_error(L, "Invalid param body");
    cpFloat radius = luaL_checknumber(L,2);
    cpVect offset = cpv(luaL_optnumber(L, 3, 0),luaL_optnumber(L, 4, 0));
    cpShape *shape = cpCircleShapeNew(body,radius,offset);
    lua_pushlightuserdata(L, shape);
    return 1;
}

static int
_shape_set_sensor(lua_State *L) {
    cpShape *shape = (cpShape *)lua_touserdata(L, 1);
    if (shape == NULL)
		return luaL_error(L, "Invalid param shape");
    cpBool sensor = (cpBool)lua_toboolean(L, 2);
    cpShapeSetSensor(shape, sensor);
    return 0;
}

static int
_shape_set_collision_type(lua_State *L) {
    cpShape *shape = (cpShape *)lua_touserdata(L, 1);
    if (shape == NULL)
		return luaL_error(L, "Invalid param shape");
    cpCollisionType type = (cpCollisionType)luaL_checkinteger(L, 2);
    cpShapeSetCollisionType(shape, type);
    return 0;
}

static int
_shape_set_filter(lua_State *L) {
    cpShape *shape = (cpShape *)lua_touserdata(L, 1);
    if (shape == NULL)
		return luaL_error(L, "Invalid param shape");
    if (!lua_istable(L, 2))
		return luaL_error(L, "Invalid param filter");
    lua_getfield(L, 2, "group");
    lua_getfield(L, 2, "categories");
    lua_getfield(L, 2, "mask");
    cpShapeFilter filter = cpShapeFilterNew(
        luaL_optnumber(L, -3, CP_NO_GROUP),
        luaL_optnumber(L, -2, ~CP_ALL_CATEGORIES),
        luaL_optnumber(L, -1, ~CP_ALL_CATEGORIES)
    );
    cpShapeSetFilter(shape, filter);
    return 0;
}

static int
_space_add_shape(lua_State *L) {
    cpSpace *space = (cpSpace *)lua_touserdata(L, 1);
    cpShape *shape = (cpShape *)lua_touserdata(L, 2);
    if (space == NULL || shape == NULL)
		return luaL_error(L, "Invalid param space or shape");
    cpSpaceAddShape(space, shape);
    return 0;
}

static int
_space_remove_shape(lua_State *L) {
    cpShape *shape = (cpShape *)lua_touserdata(L, 1);
    if (shape == NULL)
		return luaL_error(L, "Invalid param shape");
    cpSpace *space = cpShapeGetSpace(shape);
    if (space == NULL)
		return luaL_error(L, "shape not in space yet");
    cpSpaceRemoveShape(space, shape);
    return 0;
}

static int
_shape_free(lua_State *L) {
    cpShape *shape = (cpShape *)lua_touserdata(L, 1);
    if (shape == NULL)
		return luaL_error(L, "Invalid param shape");
    cpShapeFree(shape);
    return 0;
}

int
luaopen_libchipmunk(lua_State *L) {
    luaL_checkversion(L);

    lua_newtable(L);
    lua_setfield(L, LUA_REGISTRYINDEX, "chipmunk_collision_handler");
    lua_newtable(L);
    lua_setfield(L, LUA_REGISTRYINDEX, "chipmunk_collision_data");

    luaL_Reg l[] = {
        {"space_new", _space_new},
        {"space_add_collision_handler", _space_add_collision_handler},
        {"space_add_body", _space_add_body},
        {"space_remove_body", _space_remove_body},
        {"space_free", _space_free},
        {"space_step", _space_step},
        {"body_new", _body_new},
        {"body_new_kinematic", _body_new_kinematic},
        {"body_new_static", _body_new_static},
        {"body_local_to_world_vector", _body_local_to_world_vector},
        {"body_local_to_world_point", _body_local_to_world_point},
        {"body_get_position", _body_get_position},
        {"body_set_position", _body_set_position},
        {"space_reindex_shapes_for_body", _space_reindex_shapes_for_body},
        {"body_get_forward", _body_get_forward},
        {"body_get_angle", _body_get_angle},
        {"body_set_angle", _body_set_angle},
        {"body_set_velocity", _body_set_velocity},
        {"body_get_velocity", _body_get_velocity},
        {"body_free", _body_free},
        {"poly_shape_new", _poly_shape_new},
        {"circle_shape_new", _circle_shape_new},
        {"shape_set_sensor", _shape_set_sensor},
        {"shape_set_collision_type", _shape_set_collision_type},
        {"shape_set_filter", _shape_set_filter},
        {"space_add_shape", _space_add_shape},
        {"space_remove_shape", _space_remove_shape},
        {"shape_free", _shape_free},
        {NULL, NULL}
    };

    luaL_newlib(L, l);
    return 1;
}

