//==============================================================================
/*
    miPhysics core engine, C++ port of the Java/Processing library
    (github.com/mi-creative/miPhysics_Processing).

    Header-only and independent of CHAI3D. Ported subset: Vect3D, Mat, Link,
    Mass3D, Mass2DPlane, Ground3D, HapticInput3D, SpringDamper3D, Contact3D,
    PhysicalModel. ModelLoader, TopoGenerator and controllers are not ported.
*/
//==============================================================================

#ifndef MIPHYSICS_H
#define MIPHYSICS_H

#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace miphysics {

// C++11 compatible std::make_unique
template <class T, class... Args>
std::unique_ptr<T> makeUnique(Args&&... a_args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(a_args)...));
}

//------------------------------------------------------------------------------
// Vect3D
//------------------------------------------------------------------------------

struct Vect3D
{
    double x = 0.0, y = 0.0, z = 0.0;

    Vect3D() = default;
    Vect3D(double a_x, double a_y, double a_z) : x(a_x), y(a_y), z(a_z) {}

    bool operator==(const Vect3D& v) const { return x == v.x && y == v.y && z == v.z; }

    double norm() const { return std::sqrt(x * x + y * y + z * z); }

    // returns a tiny non-zero value for coincident points to avoid NaN
    double dist(const Vect3D& v) const
    {
        if (*this == v) return 1e-8;
        return std::sqrt(sqDistRaw(v));
    }

    double sqDist(const Vect3D& v) const
    {
        if (*this == v) return 1e-8;
        return sqDistRaw(v);
    }

    Vect3D& add(const Vect3D& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vect3D& sub(const Vect3D& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vect3D& mult(double f) { x *= f; y *= f; z *= f; return *this; }
    Vect3D& div(double f) { return mult(1.0 / f); }
    Vect3D& set(const Vect3D& v) { x = v.x; y = v.y; z = v.z; return *this; }
    Vect3D& set(double a_x, double a_y, double a_z) { x = a_x; y = a_y; z = a_z; return *this; }
    void reset() { x = y = z = 0.0; }

private:
    double sqDistRaw(const Vect3D& v) const
    {
        return (v.x - x) * (v.x - x) + (v.y - y) * (v.y - y) + (v.z - z) * (v.z - z);
    }
};

//------------------------------------------------------------------------------
// Module types
//------------------------------------------------------------------------------

enum class MatType  { UNDEFINED, Mass3D, Mass2DPlane, Ground3D, HapticInput3D };
enum class LinkType { UNDEFINED, SpringDamper3D, Contact3D };

//------------------------------------------------------------------------------
// Mat: material point integrated with a Verlet scheme
//------------------------------------------------------------------------------

class Mat
{
public:
    Mat(double a_mass, const Vect3D& a_pos, const Vect3D& a_posR)
        : pos(a_pos), posR(a_posR), invMass(1.0 / a_mass) {}
    virtual ~Mat() = default;

    virtual void compute() = 0;
    virtual void setGravity(const Vect3D&) {}
    virtual void setFriction(double) {}

    MatType type() const { return type_; }
    double mass() const { return 1.0 / invMass; }

    void applyExtForce(const Vect3D& f) { frc.add(f); }

    Vect3D pos;     // current position
    Vect3D posR;    // delayed (previous step) position
    Vect3D frc;     // force accumulation buffer
    double invMass;

protected:
    MatType type_ = MatType::UNDEFINED;
};

//------------------------------------------------------------------------------
// Mass3D / Mass2DPlane: inertial mats with friction and gravity
//------------------------------------------------------------------------------

class Mass3D : public Mat
{
public:
    Mass3D(double a_mass, const Vect3D& a_pos, const Vect3D& a_posR,
           double a_friction, const Vect3D& a_grav)
        : Mat(a_mass, a_pos, a_posR), fricZ_(a_friction), gFrc_(a_grav)
    {
        type_ = MatType::Mass3D;
    }

    void compute() override
    {
        tmp_ = pos;
        frc.mult(invMass);
        pos.mult(2 - invMass * fricZ_);
        posR.mult(1 - invMass * fricZ_);
        pos.sub(posR);
        pos.add(frc);
        pos.sub(gFrc_);
        posR = tmp_;
        frc.reset();
    }

    void setGravity(const Vect3D& g) override { gFrc_ = g; }
    void setFriction(double f) override { fricZ_ = f; }

private:
    double fricZ_;
    Vect3D gFrc_;
    Vect3D tmp_;
};

class Mass2DPlane : public Mat
{
public:
    Mass2DPlane(double a_mass, const Vect3D& a_pos, const Vect3D& a_posR,
                double a_friction, const Vect3D& a_grav)
        : Mat(a_mass, a_pos, a_posR), fricZ_(a_friction), gFrc_(a_grav)
    {
        type_ = MatType::Mass2DPlane;
        posR.z = pos.z;
    }

    void compute() override
    {
        tmp_ = pos;
        frc.mult(invMass);
        pos.mult(2 - invMass * fricZ_);
        posR.mult(1 - invMass * fricZ_);
        pos.sub(posR);
        pos.add(frc);
        pos.sub(gFrc_);
        pos.z = tmp_.z;     // constrain to the XY plane
        posR = tmp_;
        frc.reset();
    }

    void setGravity(const Vect3D& g) override { gFrc_ = g; }
    void setFriction(double f) override { fricZ_ = f; }

private:
    double fricZ_;
    Vect3D gFrc_;
    Vect3D tmp_;
};

//------------------------------------------------------------------------------
// Ground3D: fixed point
//------------------------------------------------------------------------------

class Ground3D : public Mat
{
public:
    explicit Ground3D(const Vect3D& a_pos) : Mat(1.0, a_pos, a_pos)
    {
        type_ = MatType::Ground3D;
    }

    void compute() override { frc.reset(); }
};

//------------------------------------------------------------------------------
// HapticInput3D: position set from the haptic device, force read back
//------------------------------------------------------------------------------

class HapticInput3D : public Mat
{
public:
    HapticInput3D(const Vect3D& a_pos, int a_smoothing)
        : Mat(1.0, a_pos, a_pos), directPos_(a_pos), smooth_(1.0 / (double)a_smoothing)
    {
        type_ = MatType::HapticInput3D;
    }

    // exponentially weighted moving average towards the device position
    void compute() override
    {
        pos.x = smooth_ * directPos_.x + (1 - smooth_) * pos.x;
        pos.y = smooth_ * directPos_.y + (1 - smooth_) * pos.y;
        pos.z = smooth_ * directPos_.z + (1 - smooth_) * pos.z;
        // force is not reset here: it accumulates until the haptic loop reads it
    }

    void applyInputPosition(const Vect3D& p) { directPos_ = p; }

    Vect3D applyOutputForce()
    {
        Vect3D out = frc;
        frc.reset();
        return out;
    }

private:
    Vect3D directPos_;
    double smooth_;
};

//------------------------------------------------------------------------------
// Link: interaction between two mats
//------------------------------------------------------------------------------

class Link
{
public:
    Link(double a_dRest, Mat* a_m1, Mat* a_m2)
        : mat1_(a_m1), mat2_(a_m2), dRest_(a_dRest) {}
    virtual ~Link() = default;

    virtual void compute() = 0;

    LinkType type() const { return type_; }
    Mat* mat1() const { return mat1_; }
    Mat* mat2() const { return mat2_; }

    void initDistances()
    {
        dist_  = mat1_->pos.dist(mat2_->pos);
        distR_ = mat1_->posR.dist(mat2_->posR);
    }

protected:
    double updateEuclidDist()
    {
        distR_ = dist_;
        dist_  = mat1_->pos.dist(mat2_->pos);
        return dist_;
    }

    // per-sample velocity of the elongation
    double getVel() const { return dist_ - distR_; }

    void applyForces(double lnkFrc)
    {
        double inv = 1.0 / dist_;
        double xp = (mat1_->pos.x - mat2_->pos.x) * inv;
        double yp = (mat1_->pos.y - mat2_->pos.y) * inv;
        double zp = (mat1_->pos.z - mat2_->pos.z) * inv;

        mat1_->frc.x += lnkFrc * xp;
        mat1_->frc.y += lnkFrc * yp;
        mat1_->frc.z += lnkFrc * zp;

        mat2_->frc.x -= lnkFrc * xp;
        mat2_->frc.y -= lnkFrc * yp;
        mat2_->frc.z -= lnkFrc * zp;
    }

    LinkType type_ = LinkType::UNDEFINED;
    Mat* mat1_;
    Mat* mat2_;
    double dist_  = 0.0;
    double distR_ = 0.0;
    double dRest_;
    double K_ = 0.0;
    double Z_ = 0.0;
};

class SpringDamper3D : public Link
{
public:
    SpringDamper3D(double a_dRest, double a_K, double a_Z, Mat* a_m1, Mat* a_m2)
        : Link(a_dRest, a_m1, a_m2)
    {
        type_ = LinkType::SpringDamper3D;
        K_ = a_K;
        Z_ = a_Z;
    }

    void compute() override
    {
        updateEuclidDist();
        applyForces(-(dist_ - dRest_) * K_ - getVel() * Z_);
    }
};

// viscoelastic force applied only when the mats are closer than dRest
class Contact3D : public Link
{
public:
    Contact3D(double a_dRest, double a_K, double a_Z, Mat* a_m1, Mat* a_m2)
        : Link(a_dRest, a_m1, a_m2)
    {
        type_ = LinkType::Contact3D;
        K_ = a_K;
        Z_ = a_Z;
    }

    void compute() override
    {
        updateEuclidDist();
        if (dist_ < dRest_)
            applyForces(-(dist_ - dRest_) * K_ - getVel() * Z_);
    }
};

//------------------------------------------------------------------------------
// PhysicalModel: owns mats and links, steps the simulation
//------------------------------------------------------------------------------

class PhysicalModel
{
public:
    explicit PhysicalModel(int a_simRate) : simRate_(a_simRate > 0 ? a_simRate : 50) {}

    int getSimRate() const { return simRate_; }
    void setSimRate(int a_rate) { simRate_ = a_rate; }

    // call once all modules have been added
    void init()
    {
        for (auto& l : links_) l->initDistances();
    }

    // Guards all model state. computeStepUnlocked() and the const accessors
    // require the caller to hold it.
    std::mutex& mutex() { return mutex_; }

    void computeStepUnlocked()
    {
        for (auto& m : mats_) m->compute();
        for (auto& l : links_) l->compute();
    }

    void computeNSteps(int a_n)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < a_n; i++) computeStepUnlocked();
    }

    void computeStep() { computeNSteps(1); }

    void setFriction(double a_fr)
    {
        friction_ = a_fr;
        for (auto& m : mats_) m->setFriction(a_fr);
    }

    void setGravity(double a_g)
    {
        gScaled_ = Vect3D(0.0, 0.0, a_g);
        for (auto& m : mats_) m->setGravity(gScaled_);
    }

    //--------------------------------------------------------------------------
    // Adding modules (velocities are given in distance units per second)
    //--------------------------------------------------------------------------

    int addGround3D(const std::string& a_name, const Vect3D& a_pos)
    {
        return addMat(a_name, makeUnique<Ground3D>(a_pos));
    }

    int addMass3D(const std::string& a_name, double a_mass, const Vect3D& a_pos, const Vect3D& a_vel)
    {
        return addMat(a_name, makeUnique<Mass3D>(a_mass, a_pos, delayedPos(a_pos, a_vel), friction_, gScaled_));
    }

    int addMass2DPlane(const std::string& a_name, double a_mass, const Vect3D& a_pos, const Vect3D& a_vel)
    {
        return addMat(a_name, makeUnique<Mass2DPlane>(a_mass, a_pos, delayedPos(a_pos, a_vel), friction_, gScaled_));
    }

    int addHapticInput3D(const std::string& a_name, const Vect3D& a_pos, int a_smoothing)
    {
        return addMat(a_name, makeUnique<HapticInput3D>(a_pos, a_smoothing));
    }

    // link names are not required to be unique
    int addSpringDamper3D(const std::string& a_name, double a_dRest, double a_K, double a_Z,
                          const std::string& a_m1, const std::string& a_m2)
    {
        return addLink(a_name, makeUnique<SpringDamper3D>(a_dRest, a_K, a_Z, matByName(a_m1), matByName(a_m2)));
    }

    int addContact3D(const std::string& a_name, double a_dRest, double a_K, double a_Z,
                     const std::string& a_m1, const std::string& a_m2)
    {
        return addLink(a_name, makeUnique<Contact3D>(a_dRest, a_K, a_Z, matByName(a_m1), matByName(a_m2)));
    }

    //--------------------------------------------------------------------------
    // Haptic interface (thread safe)
    //--------------------------------------------------------------------------

    void setHapticPosition(const std::string& a_name, const Vect3D& a_pos)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        hapticByName(a_name)->applyInputPosition(a_pos);
    }

    Vect3D getHapticForce(const std::string& a_name)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return hapticByName(a_name)->applyOutputForce();
    }

    //--------------------------------------------------------------------------
    // Accessors (caller must hold mutex() when the simulation is running)
    //--------------------------------------------------------------------------

    int getNumberOfMats() const { return (int)mats_.size(); }
    int getNumberOfLinks() const { return (int)links_.size(); }
    bool matExists(const std::string& a_name) const { return matIndex_.count(a_name) > 0; }
    const Mat* getMat(const std::string& a_name) const { return mats_[matIndex_.at(a_name)].get(); }
    const Mat* getMatAt(int a_i) const { return mats_[a_i].get(); }
    const Link* getLinkAt(int a_i) const { return links_[a_i].get(); }
    const std::string& getMatNameAt(int a_i) const { return matNames_[a_i]; }

private:
    Vect3D delayedPos(const Vect3D& a_pos, const Vect3D& a_vel) const
    {
        Vect3D p = a_pos;
        Vect3D v = a_vel;
        return p.sub(v.div(simRate_));
    }

    int addMat(const std::string& a_name, std::unique_ptr<Mat> a_mat)
    {
        if (matIndex_.count(a_name))
            throw std::invalid_argument("miphysics: mat name already exists: " + a_name);
        int idx = (int)mats_.size();
        matIndex_[a_name] = idx;
        matNames_.push_back(a_name);
        mats_.push_back(std::move(a_mat));
        return idx;
    }

    int addLink(const std::string& a_name, std::unique_ptr<Link> a_link)
    {
        int idx = (int)links_.size();
        linkNames_.push_back(a_name);
        links_.push_back(std::move(a_link));
        return idx;
    }

    Mat* matByName(const std::string& a_name)
    {
        auto it = matIndex_.find(a_name);
        if (it == matIndex_.end())
            throw std::invalid_argument("miphysics: unknown mat: " + a_name);
        return mats_[it->second].get();
    }

    HapticInput3D* hapticByName(const std::string& a_name)
    {
        auto* h = dynamic_cast<HapticInput3D*>(matByName(a_name));
        if (!h) throw std::invalid_argument("miphysics: not a haptic input: " + a_name);
        return h;
    }

    int simRate_;
    double friction_ = 0.0;
    Vect3D gScaled_;

    std::vector<std::unique_ptr<Mat>> mats_;
    std::vector<std::unique_ptr<Link>> links_;
    std::vector<std::string> matNames_;
    std::vector<std::string> linkNames_;
    std::unordered_map<std::string, int> matIndex_;

    std::mutex mutex_;
};

} // namespace miphysics

#endif // MIPHYSICS_H
