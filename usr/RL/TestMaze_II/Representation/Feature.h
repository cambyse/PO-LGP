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
    USE_CONFIG_TYPEDEFS;
    typedef std::shared_ptr<const Feature> const_feature_ptr_t;
    typedef double feature_return_value;
    typedef std::unordered_map<const_feature_ptr_t, feature_return_value> look_up_map_t;
    enum FEATURE_TYPE { ABSTRACT, CONST_FEATURE, ACTION, OBSERVATION, REWARD, AND };
    // functions
    Feature();
    virtual ~Feature();
    virtual feature_return_value evaluate(const_instance_ptr_t) const;
    virtual feature_return_value evaluate(const_instance_ptr_t, action_ptr_t, observation_ptr_t, reward_ptr_t) const;
    virtual feature_return_value evaluate(const look_up_map_t&) const;
    virtual std::string identifier() const;
    friend std::ostream& operator<<(std::ostream&, const Feature&);
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
    feature_return_value const_return_value;
    // member functions
    inline feature_return_value return_function(const feature_return_value& ret) const;
};

class BasisFeature: public Feature {
public:
    typedef std::weak_ptr<const Feature> this_ptr_t;
    virtual this_ptr_t get_this_ptr() const { return thisPtr; }
protected:
    BasisFeature() {}
    ~BasisFeature() {}
    this_ptr_t thisPtr;
    virtual void set_this_ptr(const_feature_ptr_t ptr) { thisPtr = ptr; }
};

class ConstFeature: public BasisFeature {
private:
    ConstFeature(const feature_return_value& v);
public:
    virtual ~ConstFeature();
    static const_feature_ptr_t create(const feature_return_value& v = 0);
    virtual feature_return_value evaluate(const_instance_ptr_t) const;
    virtual feature_return_value evaluate(const_instance_ptr_t, action_ptr_t, observation_ptr_t, reward_ptr_t) const;
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
    virtual feature_return_value evaluate(const_instance_ptr_t) const;
    virtual std::string identifier() const;
    static bool features_contradict(const ActionFeature& f1, const ActionFeature& f2);
    bool contradicts(const ActionFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
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
    virtual feature_return_value evaluate(const_instance_ptr_t) const;
    virtual std::string identifier() const;
    static bool features_contradict(const ObservationFeature& f1, const ObservationFeature& f2);
    bool contradicts(const ObservationFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
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
    virtual feature_return_value evaluate(const_instance_ptr_t) const;
    virtual std::string identifier() const;
    static bool features_contradict(const RewardFeature& f1, const RewardFeature& f2);
    bool contradicts(const RewardFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    const reward_ptr_t get_reward() const { return reward; }
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
    virtual feature_return_value evaluate(const_instance_ptr_t) const;
    virtual feature_return_value evaluate(const look_up_map_t&) const;
    virtual std::string identifier() const;
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    virtual bool operator<(const AndFeature& other) const;
protected:
    subfeature_container_t subfeatures;
    virtual void add_feature(const_feature_ptr_t f);
    virtual void finalize_construction();
    void check_for_contradicting_subfeatures();
    inline feature_return_value return_function(const feature_return_value& ret) const;
};

#endif /* FEATURE_H_ */
