#ifndef FEATURE_H_
#define FEATURE_H_

#include <QString>

#include <list>
#include <set>
#include <string>
#include <memory> // for shared_ptr
#include <unordered_map>

#include "../Config.h"

class Feature {
public:
    // types
    typedef std::shared_ptr<const Feature> const_feature_ptr_t;
    typedef unsigned char feature_return_t;
    USE_CONFIG_TYPEDEFS;
    typedef std::unordered_map<const_feature_ptr_t, feature_return_t> look_up_map_t;
    enum FEATURE_TYPE { ABSTRACT, CONST_FEATURE, ACTION, OBSERVATION, REWARD, AND };
    // functions
    Feature();
    virtual ~Feature();
    virtual feature_return_t evaluate(const_instance_ptr_t) const;
    virtual feature_return_t evaluate(const_instance_ptr_t, action_ptr_t, observation_ptr_t, reward_ptr_t) const;
    virtual feature_return_t evaluate(const look_up_map_t&) const;
    virtual std::string identifier() const;
    friend std::ostream& operator<<(std::ostream&, const Feature&);
    virtual operator std::string() const {return util::string_from_ostream(*this);}
    /** \brief Compare features based on their type. */
    virtual bool operator==(const Feature& other) const;
    virtual bool operator!=(const Feature& other) const final;
    virtual bool operator<(const Feature& other) const;
    virtual FEATURE_TYPE get_feature_type() const;
    virtual unsigned int get_complexity() const;
    virtual bool is_const_feature() const { return const_feature; }
    virtual bool contradicts(const Feature&) const { return false; }
protected:
    // member variables
    FEATURE_TYPE feature_type;
    unsigned int complexity;
    bool const_feature;
    feature_return_t const_return_value;
    // member functions
    virtual inline feature_return_t return_function(const feature_return_t& ret) const final;
};

class BasisFeature: public Feature {
    friend class AndFeature;

    //----typedefs----//
private:
    typedef std::weak_ptr<const Feature> self_ptr_t;
    typedef std::set<const BasisFeature *,util::deref_less<const BasisFeature *> > unique_feature_set_t;

    //----members----//
private:
    /** \brief Ensure that all created BasisFeature objects are unique.
     *
     * On construction of BasisFeature objects via static create() function a
     * new object is only created if no equal object is found within
     * unique_feature_set, otherwise a pointer to this object is returned.*/
    static unique_feature_set_t unique_feature_set;
    bool erase_from_unique_feature_set = false;

    /** \brief Pointer to this object.
     *
     * Used to return/maintain unique shared pointer. */
    self_ptr_t self_ptr;

    //----methods----//
public:
    ~BasisFeature();
protected:
    BasisFeature() {}
    static const_feature_ptr_t create(BasisFeature * f);
    virtual void erase_from_unique() final;
};

class ConstFeature: public BasisFeature {
private:
    ConstFeature(const feature_return_t& v);
public:
    virtual ~ConstFeature();
    static const_feature_ptr_t create(const feature_return_t& v = 0);
    virtual feature_return_t evaluate(const_instance_ptr_t) const;
    virtual feature_return_t evaluate(const_instance_ptr_t, action_ptr_t, observation_ptr_t, reward_ptr_t) const;
    virtual std::string identifier() const;
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
};

class ActionFeature: public BasisFeature {
private:
    ActionFeature(const action_ptr_t& a, const int& d);
public:
    virtual ~ActionFeature();
    static const_feature_ptr_t create(const action_ptr_t& a, const int& d);
    virtual feature_return_t evaluate(const_instance_ptr_t) const;
    virtual std::string identifier() const;
    static bool features_contradict(const ActionFeature& f1, const ActionFeature& f2);
    bool contradicts(const ActionFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    virtual int get_delay() const { return delay; }
protected:
    action_ptr_t action;
    int delay;
};

class ObservationFeature: public BasisFeature {
private:
    ObservationFeature(const observation_ptr_t& s, const int& d);
public:
    virtual ~ObservationFeature();
    static const_feature_ptr_t create(const observation_ptr_t& s, const int& d);
    virtual feature_return_t evaluate(const_instance_ptr_t) const;
    virtual std::string identifier() const;
    static bool features_contradict(const ObservationFeature& f1, const ObservationFeature& f2);
    bool contradicts(const ObservationFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    virtual int get_delay() const { return delay; }
protected:
    observation_ptr_t observation;
    int delay;
};

class RewardFeature: public BasisFeature {
private:
    RewardFeature(const reward_ptr_t& r, const int& d);
public:
    virtual ~RewardFeature();
    static const_feature_ptr_t create(const reward_ptr_t& r, const int& d);
    virtual feature_return_t evaluate(const_instance_ptr_t) const;
    virtual std::string identifier() const;
    static bool features_contradict(const RewardFeature& f1, const RewardFeature& f2);
    bool contradicts(const RewardFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    const reward_ptr_t get_reward() const { return reward; }
    virtual int get_delay() const { return delay; }
protected:
    reward_ptr_t reward;
    int delay;
};

class AndFeature: public Feature {
public:
    typedef std::set<const_feature_ptr_t> subfeature_container_t;
    using Feature::evaluate; // so the compiler finds them
    AndFeature();
    AndFeature(const_feature_ptr_t f);
    AndFeature(const_feature_ptr_t f1, const_feature_ptr_t f2);
    virtual ~AndFeature();
    virtual feature_return_t evaluate(const_instance_ptr_t) const;
    virtual feature_return_t evaluate(const look_up_map_t&) const;
    virtual std::string identifier() const;
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    virtual bool operator<(const AndFeature& other) const;
    virtual const subfeature_container_t& get_subfeatures() const { return subfeatures; }
protected:
    subfeature_container_t subfeatures;
    virtual void add_feature(const_feature_ptr_t f);
    virtual void finalize_construction();
    void check_for_contradicting_subfeatures();
};

#endif /* FEATURE_H_ */
