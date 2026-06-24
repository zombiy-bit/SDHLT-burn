#include "csg.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

namespace
{
    struct physics_hull_t
    {
        vec3_t rotation; // degrees, X/Y/Z
        vec3_t size;     // full extents
        vec3_t offset;   // local offset from model origin
    };

    struct queued_model_physics_t
    {
        int entitynum;
        int originalentitynum;
        int originalbrushnum;
        vec3_t origin;
        vec3_t angles;
        std::vector<physics_hull_t> hulls;
    };

    struct mat3_t
    {
        vec_t m[3][3];
    };

    static std::vector<queued_model_physics_t> g_queuedModelPhysics;

    static bool ReadTextFile(const char* path, std::string& out)
    {
        FILE* f = fopen(path, "rb");
        if (!f)
        {
            return false;
        }

        if (fseek(f, 0, SEEK_END) != 0)
        {
            fclose(f);
            return false;
        }

        const long size = ftell(f);
        if (size < 0)
        {
            fclose(f);
            return false;
        }
        if (fseek(f, 0, SEEK_SET) != 0)
        {
            fclose(f);
            return false;
        }

        out.resize((size_t)size);
        if (size > 0)
        {
            if (fread(&out[0], 1, (size_t)size, f) != (size_t)size)
            {
                fclose(f);
                out.clear();
                return false;
            }
        }

        fclose(f);
        return true;
    }

    static void SkipWhitespaceAndComments(const char*& p, const char* end)
    {
        while (p < end)
        {
            if (isspace((unsigned char)*p))
            {
                ++p;
                continue;
            }
            if (p + 1 < end && p[0] == '/' && p[1] == '/')
            {
                p += 2;
                while (p < end && *p != '\n' && *p != '\r')
                {
                    ++p;
                }
                continue;
            }
            break;
        }
    }

    struct token_t
    {
        enum type_t
        {
            END,
            WORD,
            STRING,
            LBRACE,
            RBRACE,
            COLON,
            COMMA
        } type;
        std::string text;
    };

    class physics_lexer_t
    {
    public:
        explicit physics_lexer_t(const std::string& text)
            : m_cur(text.c_str()), m_end(text.c_str() + text.size())
        {
        }

        token_t next()
        {
            SkipWhitespaceAndComments(m_cur, m_end);
            if (m_cur >= m_end)
            {
                return make(token_t::END);
            }

            const char c = *m_cur++;
            switch (c)
            {
            case '{':
                return make(token_t::LBRACE, "{");
            case '}':
                return make(token_t::RBRACE, "}");
            case ':':
                return make(token_t::COLON, ":");
            case ',':
                return make(token_t::COMMA, ",");
            case '"':
            {
                std::string s;
                while (m_cur < m_end)
                {
                    const char ch = *m_cur++;
                    if (ch == '"')
                    {
                        break;
                    }
                    if (ch == '\\' && m_cur < m_end)
                    {
                        const char esc = *m_cur++;
                        switch (esc)
                        {
                        case 'n': s.push_back('\n'); break;
                        case 'r': s.push_back('\r'); break;
                        case 't': s.push_back('\t'); break;
                        case '"': s.push_back('"'); break;
                        case '\\': s.push_back('\\'); break;
                        default: s.push_back(esc); break;
                        }
                    }
                    else
                    {
                        s.push_back(ch);
                    }
                }
                return make(token_t::STRING, s);
            }
            default:
            {
                const char* start = m_cur - 1;
                while (m_cur < m_end)
                {
                    const char ch = *m_cur;
                    if (isspace((unsigned char)ch) || ch == '{' || ch == '}' || ch == ':' || ch == ',' || ch == '"')
                    {
                        break;
                    }
                    if (ch == '/' && m_cur + 1 < m_end && m_cur[1] == '/')
                    {
                        break;
                    }
                    ++m_cur;
                }
                return make(token_t::WORD, std::string(start, (size_t)(m_cur - start)));
            }
            }
        }

        token_t peek()
        {
            const char* saved = m_cur;
            token_t t = next();
            m_cur = saved;
            return t;
        }

    private:
        token_t make(token_t::type_t type, const std::string& text = std::string())
        {
            token_t t;
            t.type = type;
            t.text = text;
            return t;
        }

        const char* m_cur;
        const char* m_end;
    };

    static bool ParseVec3FromString(const std::string& s, vec3_t out)
    {
        double x = 0.0, y = 0.0, z = 0.0;
        if (sscanf(s.c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
        {
            return false;
        }
        out[0] = (vec_t)x;
        out[1] = (vec_t)y;
        out[2] = (vec_t)z;
        return true;
    }

    static bool ReadVectorValue(physics_lexer_t& lex, vec3_t out)
    {
        token_t t = lex.next();
        if (t.type != token_t::STRING && t.type != token_t::WORD)
        {
            return false;
        }

        if (ParseVec3FromString(t.text, out))
        {
            return true;
        }

        if (t.type != token_t::WORD)
        {
            return false;
        }

        std::string combined = t.text;
        for (int i = 0; i < 2; ++i)
        {
            token_t part = lex.next();
            if (part.type == token_t::COMMA)
            {
                --i;
                continue;
            }
            if (part.type != token_t::WORD && part.type != token_t::STRING)
            {
                return false;
            }
            combined += " ";
            combined += part.text;
        }
        return ParseVec3FromString(combined, out);
    }

    static bool ParseModelPhysicsText(const std::string& text, std::vector<physics_hull_t>& hulls)
    {
        physics_lexer_t lex(text);

        while (true)
        {
            token_t t = lex.next();
            if (t.type == token_t::END)
            {
                break;
            }
            if (t.type == token_t::COMMA)
            {
                continue;
            }

            if (t.type != token_t::LBRACE)
            {
                // Block label such as 1Hull, BoxA, etc. Ignore it and look for the next brace.
                token_t next = lex.next();
                while (next.type == token_t::COMMA)
                {
                    next = lex.next();
                }
                if (next.type != token_t::LBRACE)
                {
                    continue;
                }
            }

            physics_hull_t hull;
            VectorClear(hull.rotation);
            VectorClear(hull.offset);
            VectorClear(hull.size);

            bool haveSize = false;
            bool blockOk = true;

            while (true)
            {
                token_t key = lex.next();
                if (key.type == token_t::END)
                {
                    blockOk = false;
                    break;
                }
                if (key.type == token_t::COMMA)
                {
                    continue;
                }
                if (key.type == token_t::RBRACE)
                {
                    break;
                }

                if (key.type != token_t::WORD && key.type != token_t::STRING)
                {
                    continue;
                }

                token_t sep = lex.peek();
                if (sep.type == token_t::COLON)
                {
                    lex.next();
                }

                vec3_t value;
                if (!ReadVectorValue(lex, value))
                {
                    blockOk = false;
                    break;
                }

                if (strcasecmp(key.text.c_str(), "rotation") == 0)
                {
                    VectorCopy(value, hull.rotation);
                }
                else if (strcasecmp(key.text.c_str(), "size") == 0)
                {
                    VectorCopy(value, hull.size);
                    haveSize = true;
                }
                else if (strcasecmp(key.text.c_str(), "offset") == 0)
                {
                    VectorCopy(value, hull.offset);
                }
                // Unknown keys are ignored deliberately.
            }

            if (blockOk && haveSize && fabs((double)hull.size[0]) > 0.0 && fabs((double)hull.size[1]) > 0.0 && fabs((double)hull.size[2]) > 0.0)
            {
                hulls.push_back(hull);
            }
        }

        return !hulls.empty();
    }

    static void MatrixIdentity(mat3_t& m)
    {
        memset(&m, 0, sizeof(m));
        m.m[0][0] = 1;
        m.m[1][1] = 1;
        m.m[2][2] = 1;
    }

    static void MatrixMultiply(const mat3_t& a, const mat3_t& b, mat3_t& out)
    {
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                out.m[r][c] =
                    a.m[r][0] * b.m[0][c] +
                    a.m[r][1] * b.m[1][c] +
                    a.m[r][2] * b.m[2][c];
            }
        }
    }

    // Conventional X/Y/Z Euler rotation order for the physics file and entity angles.
    // Apply X, then Y, then Z.
    static void AngleVectorsGoldSrc(const vec3_t angles, vec3_t forward, vec3_t right, vec3_t up)
    {
        const vec_t pitch = angles[0] * (vec_t)(Q_PI / 180.0);
        const vec_t yaw = angles[1] * (vec_t)(Q_PI / 180.0);
        const vec_t roll = angles[2] * (vec_t)(Q_PI / 180.0);

        const vec_t sp = (vec_t)sin(pitch);
        const vec_t cp = (vec_t)cos(pitch);
        const vec_t sy = (vec_t)sin(yaw);
        const vec_t cy = (vec_t)cos(yaw);
        const vec_t sr = (vec_t)sin(roll);
        const vec_t cr = (vec_t)cos(roll);

        if (forward)
        {
            forward[0] = cp * cy;
            forward[1] = cp * sy;
            forward[2] = -sp;
        }

        if (right)
        {
            right[0] = -sr * sp * cy + -cr * -sy;
            right[1] = -sr * sp * sy + -cr * cy;
            right[2] = -sr * cp;
        }

        if (up)
        {
            up[0] = cr * sp * cy + -sr * -sy;
            up[1] = cr * sp * sy + -sr * cy;
            up[2] = cr * cp;
        }
    }

    static mat3_t MatrixFromGoldSrcAngles(const vec3_t angles)
    {
        vec3_t f, r, u;
        AngleVectorsGoldSrc(angles, f, r, u);

        mat3_t m;
        m.m[0][0] = f[0];  m.m[0][1] = r[0];  m.m[0][2] = u[0];
        m.m[1][0] = f[1];  m.m[1][1] = r[1];  m.m[1][2] = u[1];
        m.m[2][0] = f[2];  m.m[2][1] = r[2];  m.m[2][2] = u[2];
        return m;
    }

    static void MatrixTransformVector(const mat3_t& m, const vec3_t in, vec3_t out)
    {
        out[0] = m.m[0][0] * in[0] + m.m[0][1] * in[1] + m.m[0][2] * in[2];
        out[1] = m.m[1][0] * in[0] + m.m[1][1] * in[1] + m.m[1][2] * in[2];
        out[2] = m.m[2][0] * in[0] + m.m[2][1] * in[1] + m.m[2][2] * in[2];
    }

    static bool IsPointModelEntity(const entity_t* ent)
    {
        const char* model = ValueForKey(ent, "model");
        if (!model || !*model)
        {
            return false;
        }
        if (model[0] == '*')
        {
            return false;
        }
        return true;
    }

    static std::string BuildPhysicsPath(const char* model)
    {
        std::string path(model);
        for (size_t i = 0; i < path.size(); ++i)
        {
            if (path[i] == '\\')
            {
                path[i] = '/';
            }
        }

        const size_t slash = path.find_last_of('/');
        const size_t dot = path.find_last_of('.');
        if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
        {
            path.erase(dot);
        }

        path += "_physics.txt";
        return path;
    }

    static bool MakeBoxFace(side_t& side, const vec3_t p0, const vec3_t p1, const vec3_t p2)
    {
        VectorCopy(p0, side.planepts[0]);
        VectorCopy(p1, side.planepts[1]);
        VectorCopy(p2, side.planepts[2]);
        side.bevel = false;
        side.td.txcommand = 0;
        safe_strncpy(side.td.name, "NULL", sizeof(side.td.name));
        memset(&side.td.vects, 0, sizeof(side.td.vects));
        return true;
    }

    static void TransformByBasis(const vec3_t in,
        const vec3_t forward,
        const vec3_t right,
        const vec3_t up,
        vec3_t out)
    {
        out[0] = forward[0] * in[0] + right[0] * in[1] + up[0] * in[2];
        out[1] = forward[1] * in[0] + right[1] * in[1] + up[1] * in[2];
        out[2] = forward[2] * in[0] + right[2] * in[1] + up[2] * in[2];
    }

    static void MakeFacePoints(const vec3_t faceCenter,
        const vec3_t tangentA,
        const vec3_t tangentB,
        vec3_t p0, vec3_t p1, vec3_t p2)
    {
        // CCW winding as seen from outside face
        VectorCopy(faceCenter, p0);
        VectorCopy(faceCenter, p1);
        VectorCopy(faceCenter, p2);

        VectorAdd(p0, tangentA, p0);
        VectorAdd(p0, tangentB, p0);

        VectorAdd(p1, tangentA, p1);
        VectorSubtract(p1, tangentB, p1);

        VectorSubtract(p2, tangentA, p2);
        VectorSubtract(p2, tangentB, p2);
    }

    static void OrthonormalizeAxes(vec3_t xAxis, vec3_t yAxis, vec3_t zAxis)
    {
        if (!VectorNormalize(xAxis))
        {
            VectorClear(xAxis);
            xAxis[0] = 1;
        }

        vec_t dotXY = DotProduct(yAxis, xAxis);
        vec3_t tmp;
        VectorScale(xAxis, dotXY, tmp);
        VectorSubtract(yAxis, tmp, yAxis);

        if (!VectorNormalize(yAxis))
        {
            VectorClear(yAxis);
            yAxis[1] = 1;
        }

        CrossProduct(xAxis, yAxis, zAxis);
        if (!VectorNormalize(zAxis))
        {
            VectorClear(zAxis);
            zAxis[2] = 1;
        }

        CrossProduct(zAxis, xAxis, yAxis);
        if (!VectorNormalize(yAxis))
        {
            VectorClear(yAxis);
            yAxis[1] = 1;
        }
    }

    static bool BuildPhysicsBrushSides(const queued_model_physics_t& item,
        const physics_hull_t& hull,
        side_t sides[6])
    {
        const vec_t hx = (vec_t)fabs((double)hull.size[0]) * 0.5f;
        const vec_t hy = (vec_t)fabs((double)hull.size[1]) * 0.5f;
        const vec_t hz = (vec_t)fabs((double)hull.size[2]) * 0.5f;

        if (hx <= 0.0f || hy <= 0.0f || hz <= 0.0f)
        {
            return false;
        }

        vec3_t ef, er, eu;
        vec3_t hf, hr, hu;
        AngleVectorsGoldSrc(item.angles, ef, er, eu);
        AngleVectorsGoldSrc(hull.rotation, hf, hr, hu);

        mat3_t entityMat = MatrixFromGoldSrcAngles(item.angles);

        vec3_t xAxis, yAxis, zAxis;
        TransformByBasis(hf, ef, er, eu, xAxis);
        TransformByBasis(hr, ef, er, eu, yAxis);
        TransformByBasis(hu, ef, er, eu, zAxis);

        OrthonormalizeAxes(xAxis, yAxis, zAxis);

        VectorScale(xAxis, hx, xAxis);
        VectorScale(yAxis, hy, yAxis);
        VectorScale(zAxis, hz, zAxis);

        vec3_t centerLocal, centerWorld;
        MatrixTransformVector(entityMat, hull.offset, centerLocal);
        VectorAdd(item.origin, centerLocal, centerWorld);

        vec3_t faceCenter, p0, p1, p2;

        // +X
        VectorAdd(centerWorld, xAxis, faceCenter);
        MakeFacePoints(faceCenter, yAxis, zAxis, p0, p1, p2);
        MakeBoxFace(sides[0], p0, p1, p2);

        // -X
        VectorSubtract(centerWorld, xAxis, faceCenter);
        MakeFacePoints(faceCenter, yAxis, zAxis, p0, p1, p2);
        MakeBoxFace(sides[1], p0, p1, p2);

        // +Y
        VectorAdd(centerWorld, yAxis, faceCenter);
        MakeFacePoints(faceCenter, xAxis, zAxis, p0, p1, p2);
        MakeBoxFace(sides[2], p0, p1, p2);

        // -Y
        VectorSubtract(centerWorld, yAxis, faceCenter);
        MakeFacePoints(faceCenter, xAxis, zAxis, p0, p1, p2);
        MakeBoxFace(sides[3], p0, p1, p2);

        // +Z
        VectorAdd(centerWorld, zAxis, faceCenter);
        MakeFacePoints(faceCenter, xAxis, yAxis, p0, p1, p2);
        MakeBoxFace(sides[4], p0, p1, p2);

        // -Z
        VectorSubtract(centerWorld, zAxis, faceCenter);
        MakeFacePoints(faceCenter, xAxis, yAxis, p0, p1, p2);
        MakeBoxFace(sides[5], p0, p1, p2);

        return true;
    }

    static bool BuildGeneratedBrush(brush_t& brush,
        side_t sides[6],
        int brushIndex,
        const queued_model_physics_t& item,
        const physics_hull_t& hull,
        int hullIndex)
    {
        memset(&brush, 0, sizeof(brush));
        brush.originalentitynum = item.originalentitynum;
        brush.originalbrushnum = hullIndex;
        brush.entitynum = 0;
        brush.brushnum = brushIndex;
        brush.firstside = 0;
        brush.numsides = 6;
        brush.noclip = 0;
        brush.cliphull = 0;
        brush.bevel = false;
        brush.detaillevel = 0;
        brush.chopdown = 0;
        brush.chopup = 0;
        brush.clipnodedetaillevel = 0;
        brush.coplanarpriority = 0;
        for (int i = 0; i < NUM_HULLS; ++i)
        {
            brush.hullshapes[i] = NULL;
        }
        brush.contents = CONTENTS_SOLID;

        return BuildPhysicsBrushSides(item, hull, sides);
    }

} // anonymous namespace

void QueueModelPhysicsForEntity(entity_t* mapent, int entitynum)
{
    if (!mapent)
    {
        return;
    }

    if (IntForKey(mapent, "Collide") != 1)
    {
        return;
    }

    if (!IsPointModelEntity(mapent))
    {
        return;
    }

    const char* model = ValueForKey(mapent, "model");
    const std::string physicsPath = BuildPhysicsPath(model);

    std::string fileData;
    if (!ReadTextFile(physicsPath.c_str(), fileData))
    {
        Warning("Entity %i: missing physics file for model '%s' (%s)", entitynum, model, physicsPath.c_str());
        return;
    }

    std::vector<physics_hull_t> hulls;
    if (!ParseModelPhysicsText(fileData, hulls))
    {
        Warning("Entity %i: physics file for model '%s' is empty or invalid (%s)", entitynum, model, physicsPath.c_str());
        return;
    }

    queued_model_physics_t item;
    item.entitynum = entitynum;
    item.originalentitynum = entitynum;
    item.originalbrushnum = 0;
    GetVectorForKey(mapent, "origin", item.origin);
    GetVectorForKey(mapent, "angles", item.angles);
    item.hulls = hulls;
    g_queuedModelPhysics.push_back(item);
}

void FinalizeQueuedModelPhysicsBrushes()
{
    if (g_queuedModelPhysics.empty())
    {
        return;
    }

    int totalBrushes = 0;
    int totalSides = 0;
    for (size_t i = 0; i < g_queuedModelPhysics.size(); ++i)
    {
        totalBrushes += (int)g_queuedModelPhysics[i].hulls.size();
        totalSides += (int)g_queuedModelPhysics[i].hulls.size() * 6;
    }

    if (totalBrushes <= 0)
    {
        g_queuedModelPhysics.clear();
        return;
    }

    hlassume(g_numentities > 0, assume_MAX_MAP_ENTITIES);
    hlassume(g_nummapbrushes + totalBrushes <= MAX_MAP_BRUSHES, assume_MAX_MAP_BRUSHES);
    hlassume(g_numbrushsides + totalSides <= MAX_MAP_SIDES, assume_MAX_MAP_SIDES);

    entity_t* worldspawn = &g_entities[0];
    const int worldBrushCount = worldspawn->numbrushes;

    int worldSideCount = 0;
    for (int i = 0; i < worldBrushCount; ++i)
    {
        worldSideCount += g_mapbrushes[i].numsides;
    }

    const int tailBrushCount = g_nummapbrushes - worldBrushCount;
    const int tailSideCount = g_numbrushsides - worldSideCount;

    // Make room in both the brush and side arrays for the generated collision brushes.
    if (tailBrushCount > 0)
    {
        memmove(g_mapbrushes + worldBrushCount + totalBrushes,
            g_mapbrushes + worldBrushCount,
            sizeof(brush_t) * tailBrushCount);
    }
    if (tailSideCount > 0)
    {
        memmove(g_brushsides + worldSideCount + totalSides,
            g_brushsides + worldSideCount,
            sizeof(side_t) * tailSideCount);
    }

    int brushCursor = worldBrushCount;
    int sideCursor = worldSideCount;

    for (size_t i = 0; i < g_queuedModelPhysics.size(); ++i)
    {
        const queued_model_physics_t& item = g_queuedModelPhysics[i];

        for (size_t h = 0; h < item.hulls.size(); ++h)
        {
            brush_t* brush = &g_mapbrushes[brushCursor];
            side_t* sides = &g_brushsides[sideCursor];

            if (!BuildGeneratedBrush(*brush, sides, brushCursor, item, item.hulls[h], (int)h))
            {
                ++brushCursor;
                continue;
            }

            brush->firstside = sideCursor;

            CreateBrush(brushCursor);

            ++brushCursor;
            sideCursor += 6;
        }
    }

    worldspawn->numbrushes += totalBrushes;
    for (int i = 1; i < g_numentities; ++i)
    {
        g_entities[i].firstbrush += totalBrushes;
    }

    g_nummapbrushes += totalBrushes;
    g_numbrushsides += totalSides;

    g_queuedModelPhysics.clear();
}
