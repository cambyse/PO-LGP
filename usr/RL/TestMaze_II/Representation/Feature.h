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
    friend class AndFeature;

    //----types/classes----/
public:
    typedef std::shared_ptr<const Feature> const_feature_ptr_t;
    typedef double feature_return_t;
    USE_CONFIG_TYPEDEFS;
    enum FEATURE_TYPE { ABSTRACT, CONST_FEATURE, ACTION, OBSERVATION, REWARD, BUTTON_ACTION, AND };
    //class look_up_map_t: public std::unordered_map<const_feature_ptr_t, feature_return_t> { // derive from map
    class look_up_map_t: public f_ptr_set_t {                                               // derive from set
    public:
        virtual void insert_feature(f_ptr_t,f_ret_t);
        virtual void erase_feature(f_ptr_t);
        virtual std::vector<f_ptr_t> get_list_of_features() const;
    };
protected:
    typedef unsigned short intern_feature_return_t;

    //----methods----//
public:
    Feature();
    virtual ~Feature();
    /** Evaluate feature on instance. @param ins Instance to evaluate feature
     * on. */
    virtual feature_return_t evaluate(const_instance_ptr_t ins) const final;
    /** Evaluate feature on instance with appended action-observation-reward
     * triplet. If the basis-instance is
     * \f$(\ldots,(a_{-1},o_{-1},r_{-1}),(a_{0},o_{0},r_{0}))\f$ the feature is
     * effectively evaluated on
     * \f$(\ldots,(a_{-1},o_{-1},r_{-1}),(a_{0},o_{0},r_{0}),(a,o,r))\f$ @param
     * ins Basis-instance the new triplet is appended to @param a action @param
     * o observation @param r reward.  */
    virtual feature_return_t evaluate(const_instance_ptr_t ins, action_ptr_t a, observation_ptr_t o, reward_ptr_t r) const final;
    virtual feature_return_t evaluate(const look_up_map_t&) const final;
    virtual std::string identifier() const;
    friend std::ostream& operator<<(std::ostream&, const Feature&);
    virtual operator std::string() const final {return util::string_from_ostream(*this);}
    /** \brief Compare features based on their type. */
    virtual bool operator==(const Feature& other) const;
    virtual bool operator!=(const Feature& other) const final;
    virtual bool operator<(const Feature& other) const;
    virtual FEATURE_TYPE get_feature_type() const final;
    virtual unsigned int get_complexity() const final;
    virtual bool is_const_feature() const final { return const_feature; }
    virtual bool contradicts(const Feature&) const { return false; }
    virtual feature_return_t return_function(const intern_feature_return_t& ret) const final;
protected:
    virtual intern_feature_return_t intern_evaluate(const_instance_ptr_t ins) const;
    virtual intern_feature_return_t intern_evaluate(const_instance_ptr_t ins, action_ptr_t a, observation_ptr_t o, reward_ptr_t r) const;
    virtual intern_feature_return_t intern_evaluate(const look_up_map_t&) const;
private:

    //----variables----//
protected:
    FEATURE_TYPE feature_type;
    unsigned int complexity;
    bool const_feature;
    intern_feature_return_t const_return_value;
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
    virtual intern_feature_return_t intern_evaluate(const look_up_map_t&) const override final;
    BasisFeature() {}
    static const_feature_ptr_t create(BasisFeature * f);
    virtual void erase_from_unique() final;
};


class ConstFeature: public BasisFeature {
private:
    ConstFeature(const intern_feature_return_t& v);
protected:
    virtual intern_feature_return_t intern_evaluate(const_instance_ptr_t) const override;
    virtual intern_feature_return_t intern_evaluate(const_instance_ptr_t, action_ptr_t, observation_ptr_t, reward_ptr_t) const override;
public:
    virtual ~ConstFeature();
    static const_feature_ptr_t create(const intern_feature_return_t& v = 0);
    virtual std::string identifier() const override;
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
};


class ActionFeature: public BasisFeature {
private:
    ActionFeature(const action_ptr_t& a, const int& d);
protected:
    virtual intern_feature_return_t intern_evaluate(const_instance_ptr_t) const override;
public:
    virtual ~ActionFeature();
    static const_feature_ptr_t create(const action_ptr_t& a, const int& d);
    virtual std::string identifier() const override;
    static bool features_contradict(const ActionFeature& f1, const ActionFeature& f2);
    bool contradicts(const ActionFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    virtual int get_delay() const final { return delay; }
protected:
    action_ptr_t action;
    int delay;
};


/** Is active if a specific button was pressed by an action. */
class ButtonActionFeature: public BasisFeature {
private:
    ButtonActionFeature(const int& idx, const int& d, const bool& n);
protected:
    virtual intern_feature_return_t intern_evaluate(const_instance_ptr_t) const override;
public:
    virtual ~ButtonActionFeature();
    static const_feature_ptr_t create(const int& idx, const int& d, const bool& n);
    virtual std::string identifier() const override;
    static bool features_contradict(const ButtonActionFeature& f1, const ButtonActionFeature& f2);
    bool contradicts(const ButtonActionFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    virtual int get_delay() const final { return delay; }
protected:
    bool negation;
    int button_idx;
    int delay;
};


class ObservationFeature: public BasisFeature {
private:
    ObservationFeature(const observation_ptr_t& s, const int& d);
protected:
    virtual intern_feature_return_t intern_evaluate(const_instance_ptr_t) const override;
public:
    virtual ~ObservationFeature();
    static const_feature_ptr_t create(const observation_ptr_t& s, const int& d);
    virtual std::string identifier() const override;
    static bool features_contradict(const ObservationFeature& f1, const ObservationFeature& f2);
    bool contradicts(const ObservationFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    virtual int get_delay() const final { return delay; }
protected:
    observation_ptr_t observation;
    int delay;
};


class RewardFeature: public BasisFeature {
private:
    RewardFeature(const reward_ptr_t& r, const int& d);
protected:
    virtual intern_feature_return_t intern_evaluate(const_instance_ptr_t) const override;
public:
    virtual ~RewardFeature();
    static const_feature_ptr_t create(const reward_ptr_t& r, const int& d);
    virtual std::string identifier() const override;
    static bool features_contradict(const RewardFeature& f1, const RewardFeature& f2);
    bool contradicts(const RewardFeature& f) const { return features_contradict(*this,f); }
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    virtual const reward_ptr_t get_reward() const final { return reward; }
    virtual int get_delay() const final { return delay; }
protected:
    reward_ptr_t reward;
    int delay;
};


class AndFeature: public Feature {
public:
    typedef f_set_t subfeature_set_t;
    using Feature::intern_evaluate; // so the compiler finds them
    AndFeature();
    AndFeature(const_feature_ptr_t f);
    AndFeature(const_feature_ptr_t f1, const_feature_ptr_t f2);
    AndFeature(const_feature_ptr_t f1, const_feature_ptr_t f2, const_feature_ptr_t f3);
    virtual ~AndFeature();
    virtual intern_feature_return_t intern_evaluate(const_instance_ptr_t) const override;
    virtual intern_feature_return_t intern_evaluate(const look_up_map_t&) const override;
    virtual std::string identifier() const override;
    virtual bool operator==(const Feature& other) const override;
    virtual bool operator<(const Feature& other) const override;
    virtual bool operator<(const AndFeature& other) const;
    virtual const subfeature_set_t& get_subfeatures() const final { return subfeatures; }
protected:
    subfeature_set_t subfeatures;
    virtual void add_feature(const_feature_ptr_t f) final;
    virtual void finalize_construction() final;
    virtual void check_for_contradicting_subfeatures() final;
};

#endif /* FEATURE_H_ */
