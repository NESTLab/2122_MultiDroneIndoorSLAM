#ifndef DIFFUSION_QTUSER_FUNCTIONS_H
#define DIFFUSION_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CDiffusionQTUserFunctions : public CQTOpenGLUserFunctions {

public:

    void DrawInWorld() override;

    inline static CVector3 Vector2ToVector3(CVector2 c_original, Real f_offset=0){
        return {c_original.GetX(), c_original.GetY(), f_offset};
    }

    inline void DrawOffsetRay(CVector3 c_start, CVector3 c_end, CColor c_color){
        CVector3 cHeight;
        // make height alittle smaller so it is drawn above the ground
        cHeight.SetZ(c_end.GetZ()-.002);
        // draw ray
        DrawRay(CRay3(c_start, c_end), c_color);
        // draw ray along the ground
        DrawRay(CRay3(c_start-cHeight, c_end-cHeight), c_color);
        // draw ray height
        DrawRay(CRay3(c_end, c_end-cHeight), c_color);
    }
};

#endif
