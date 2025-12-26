// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/Math/MathDefs.h"
#include "Urho3D/Math/Vector3.h"

#include <EASTL/hash_map.h>

#include <cmath>


namespace Urho3D
{

Vector3 MakePerpendicular(const Vector3& normal)
{
    // Pick an axis least aligned with normal to avoid degeneracy.
    const Vector3 axis = (Abs(normal.x_) > Abs(normal.z_)) ? Vector3::FORWARD : Vector3::RIGHT;
    const Vector3 perp = normal.CrossProduct(axis);
    if (perp.LengthSquared() > M_EPSILON)
        return perp.Normalized();

    // Fallback for pathological cases.
    return normal.CrossProduct(Vector3::UP).NormalizedOrDefault(Vector3::RIGHT);
}

ModelVertex LerpModelVertex(const ModelVertex& a, const ModelVertex& b, float t)
{
    ModelVertex result;

    result.position_ = a.position_.Lerp(b.position_, t);

    if (a.HasNormal() || b.HasNormal())
    {
        const Vector3 normalA = a.GetNormal();
        const Vector3 normalB = b.GetNormal();
        result.SetNormal(normalA.Lerp(normalB, t).Normalized());
    }

    if (a.HasTangent() || b.HasTangent())
    {
        const Vector3 tangentA = a.GetTangent();
        const Vector3 tangentB = b.GetTangent();
        Vector3 tangent3 = tangentA.Lerp(tangentB, t);

        const float tangentLenSq = tangent3.LengthSquared();
        if (tangentLenSq > M_EPSILON)
        {
            tangent3 /= Sqrt(tangentLenSq);
            if (result.HasNormal())
            {
                const Vector3 normal = result.GetNormal();
                const float dotNT = normal.DotProduct(tangent3);
                tangent3 -= normal * dotNT;
                const float orthoLenSq = tangent3.LengthSquared();
                if (orthoLenSq > M_EPSILON)
                    tangent3 /= Sqrt(orthoLenSq);
                else
                    tangent3 = MakePerpendicular(normal);
            }
        }
        else if (result.HasNormal())
        {
            tangent3 = MakePerpendicular(result.GetNormal());
        }

        const float handedness = (t < 0.5f) ? a.tangent_.w_ : b.tangent_.w_;
        result.tangent_ = Vector4(tangent3, handedness);
    }

    if (a.HasBinormal() || b.HasBinormal())
    {
        const Vector3 binormalA = a.binormal_.ToVector3();
        const Vector3 binormalB = b.binormal_.ToVector3();
        result.binormal_ = binormalA.Lerp(binormalB, t).Normalized().ToVector4();
    }

    for (unsigned i = 0; i < ModelVertex::MaxUVs; ++i)
        result.uv_[i] = a.uv_[i].Lerp(b.uv_[i], t);

    for (unsigned i = 0; i < ModelVertex::MaxColors; ++i)
    {
        const Color colorA = a.GetColor(i);
        const Color colorB = b.GetColor(i);
        result.SetColor(i, colorA.Lerp(colorB, t));
    }

    return result;
}

void MakeAxesFromNormal(const Vector3& normal, Vector3& axisU, Vector3& axisV)
{
    const Vector3 unitNormal = normal.NormalizedOrDefault(Vector3::UP);
    axisU = MakePerpendicular(unitNormal);
    axisV = unitNormal.CrossProduct(axisU).NormalizedOrDefault(Vector3::UP);
}

IntVector3 QuantizePosition(const Vector3& p, float invEps)
{
    return IntVector3{
        static_cast<int>(std::floor(p.x_ * invEps + 0.5f)),
        static_cast<int>(std::floor(p.y_ * invEps + 0.5f)),
        static_cast<int>(std::floor(p.z_ * invEps + 0.5f))};
}

} // namespace Urho3D
