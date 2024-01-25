#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Superclass of all emitters
 */
class AreaLight : public Emitter {
   private:
    Color3f m_color;

   public:
    AreaLight(const PropertyList &props) {
        m_color = props.getColor("radiance");
    }

    std::string toString() const { return "Area[]"; }
};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END