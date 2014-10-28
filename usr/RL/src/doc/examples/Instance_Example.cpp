#include "../Representation/Instance.h"
using util::INVALID;

#include "../debug.h"

// The structure created by the folowing code looks like this:
//
//                                        /-9-> >-10->
//                                     >-/
//    >-8-> >-7-> >-6-> >-0-> >-2-> >-3-> >-4-> >-5->
//              /->
//   12-> >-11-/
//
//   Note that InstanceIt iterators only iterate along properly connected lines
//   whereas ConstInstanceIt iterators also pass over forks such as 11->6. Also
//   note that iterating forward from 3 will pass to 4, while iterting back both
//   from 9 as well as 4 will pass to three (and similar for 11, 7, and 6).

int main(int, char **)
{
    Instance *i1, *i2, *i3, *i4, *i5;
    i1 = Instance::create(Action::STAY,0,0);

    i1->append_instance(Action::STAY,1,0);
    i1->append_instance(Action::STAY,2,1);
    i2 = i1->append_instance(Action::STAY,3,0);
    i1->append_instance(Action::STAY,4,0);
    i1->append_instance(Action::STAY,5,0);

    i3 = i1->prepend_instance(Action::STAY,6,0);
    i1->prepend_instance(Action::STAY,7,0);
    i1->prepend_instance(Action::STAY,8,0);


    i4 = Instance::create(Action::STAY,9,0,i2);
    i4->append_instance(Action::STAY,10,0);

    i5 = Instance::create(Action::STAY,11,0,nullptr,i3);
    i5->prepend_instance(Action::STAY,12,0);

    i1->set_container();
    i2->set_container();
    i3->set_container();
    i4->set_container();
    i5->set_container();

    DEBUG_OUT(0,"Main line forward (const)");
    for(ConstInstanceIt constIt = i1->const_first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Main line forward (non-const)");
    for(InstanceIt constIt = i1->first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Main line backwards (non-const)");
    for(InstanceIt constIt = i1->last(); constIt!=INVALID; --constIt) {
        DEBUG_OUT(0,*constIt);
    }

    DEBUG_OUT(0,"Pre-line forward (non-const)");
    for(InstanceIt constIt = i5->first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Pre-line forward (const)");
    for(ConstInstanceIt constIt = i5->const_first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Pre-line backwards (const)");
    for(ConstInstanceIt constIt = i5->const_last(); constIt!=INVALID; --constIt) {
        DEBUG_OUT(0,*constIt);
    }

    DEBUG_OUT(0,"Post-line forward (const)");
    for(ConstInstanceIt constIt = i4->const_first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Post-line backwards (const)");
    for(ConstInstanceIt constIt = i4->const_last(); constIt!=INVALID; --constIt) {
        DEBUG_OUT(0,*constIt);
    }

    DEBUG_OUT(0,"non-const Lengths:");
    DEBUG_OUT(0,"i1 to non-const first: " << i1->it().length_to_first());
    DEBUG_OUT(0,"i1 to non-const last: " << i1->it().length_to_last());
    DEBUG_OUT(0,"i2 to non-const first: " << i2->it().length_to_first());
    DEBUG_OUT(0,"i2 to non-const last: " << i2->it().length_to_last());
    DEBUG_OUT(0,"i3 to non-const first: " << i3->it().length_to_first());
    DEBUG_OUT(0,"i3 to non-const last: " << i3->it().length_to_last());
    DEBUG_OUT(0,"i4 to non-const first: " << i4->it().length_to_first());
    DEBUG_OUT(0,"i4 to non-const last: " << i4->it().length_to_last());
    DEBUG_OUT(0,"i5 to non-const first: " << i5->it().length_to_first());
    DEBUG_OUT(0,"i5 to non-const last: " << i5->it().length_to_last());

    DEBUG_OUT(0,"const Lengths:");
    DEBUG_OUT(0,"i1 to const first: " << i1->const_it().length_to_first());
    DEBUG_OUT(0,"i1 to const last: " << i1->const_it().length_to_last());
    DEBUG_OUT(0,"i2 to const first: " << i2->const_it().length_to_first());
    DEBUG_OUT(0,"i2 to const last: " << i2->const_it().length_to_last());
    DEBUG_OUT(0,"i3 to const first: " << i3->const_it().length_to_first());
    DEBUG_OUT(0,"i3 to const last: " << i3->const_it().length_to_last());
    DEBUG_OUT(0,"i4 to const first: " << i4->const_it().length_to_first());
    DEBUG_OUT(0,"i4 to const last: " << i4->const_it().length_to_last());
    DEBUG_OUT(0,"i5 to const first: " << i5->const_it().length_to_first());
    DEBUG_OUT(0,"i5 to const last: " << i5->const_it().length_to_last());

    delete i4;
    delete i5;

    DEBUG_OUT(0,"Iterate through main line after deleting i4 and i5");
    for(ConstInstanceIt constIt = i1->const_first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }

    DEBUG_OUT(0,"Trying to iterate from i5 (Segfault)");
    for(ConstInstanceIt constIt = i5->const_first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }

    return 0;
}
