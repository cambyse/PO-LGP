#ifndef BUTTONACTION_H_
#define BUTTONACTION_H_

#include <representation/AbstractAction.h>

#include <vector>

class ButtonAction: public AbstractAction {
    //----typedefs/classes----//

    //     NONE      //

    //----members----//
protected:
    int size;
    std::vector<bool> action;

    //----methods----//
public:
    ButtonAction(int s = 1, std::vector<bool> a = std::vector<bool>());
    virtual ~ButtonAction() override = default;
    /** \brief Defined explicitly because the size of button array needs to be
     * given. */
    virtual Iterator begin() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractAction &other) const override;
    virtual bool operator<(const AbstractAction &other) const override;
    virtual const std::string print() const override;
    std::vector<bool> get_array() const { return action; }
};

#endif /* BUTTONACTION_H_ */
