#include <nori/emitter.h>
#include <nori/mesh.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Superclass of all emitters
 */
class AreaLight : public Emitter {
   private:
    Color3f m_radiance;

   public:
    AreaLight(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }
    Color3f getRadiance() { return m_radiance; }

    std::string toString() const { return "Area[]"; }

    EClassType getClassType() const { return EEmitter; }
};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END