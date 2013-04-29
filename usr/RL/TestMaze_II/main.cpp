#include "testmaze_ii.h"
#include "BatchMaze.h"
#include "Data.h"

//#define BATCH_MODE

#include <QtGui>
#include <QApplication>

#include "Representation/Representation.h"
using util::INVALID;

#include "debug.h"

int main(int argc, char *argv[])
{
    instance_t *i1, *i2, *i3, *i4, *i5;
    i1 = instance_t::create(action_t::STAY,0,0);

    i1->append_instance(action_t::STAY,0,1);
    i1->append_instance(action_t::STAY,0,2);
    i2 = i1->append_instance(action_t::STAY,1,0);
    i1->append_instance(action_t::STAY,2,0);
    i1->append_instance(action_t::STAY,3,0);

    i3 = i1->prepend_instance(action_t::STAY,4,0);
    i1->prepend_instance(action_t::STAY,5,0);
    i1->prepend_instance(action_t::STAY,6,0);


    i4 = instance_t::create(action_t::STAY,7,0,i2);
    i4->append_instance(action_t::STAY,8,0);

    i5 = instance_t::create(action_t::STAY,9,0,nullptr,i3);
    i5->prepend_instance(action_t::STAY,10,0);

    i1->set_container();
    i2->set_container();
    i3->set_container();
    i4->set_container();
    i5->set_container();

    DEBUG_OUT(0,"Main line forward (const)");
    for(const_instanceIt_t constIt = i1->const_first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Main line forward (non-const)");
    for(instanceIt_t constIt = i1->first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Main line backwards (non-const)");
    for(instanceIt_t constIt = i1->last(); constIt!=INVALID; --constIt) {
        DEBUG_OUT(0,*constIt);
    }

    DEBUG_OUT(0,"Pre-line forward (non-const)");
    for(instanceIt_t constIt = i5->first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Pre-line forward (const)");
    for(const_instanceIt_t constIt = i5->const_first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Pre-line backwards (const)");
    for(const_instanceIt_t constIt = i5->const_last(); constIt!=INVALID; --constIt) {
        DEBUG_OUT(0,*constIt);
    }

    DEBUG_OUT(0,"Post-line forward (const)");
    for(const_instanceIt_t constIt = i4->const_first(); constIt!=INVALID; ++constIt) {
        DEBUG_OUT(0,*constIt);
    }
    DEBUG_OUT(0,"Post-line backwards (const)");
    for(const_instanceIt_t constIt = i4->const_last(); constIt!=INVALID; --constIt) {
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

    return 0;

    // seed random generator
    srand(time(nullptr));
    srand48(time(nullptr));

#ifdef BATCH_MODE
    BatchMaze batchmaze;
    return batchmaze.run(argc,argv);
#else
    QApplication a(argc, argv);
    TestMaze_II w;
    w.show();
    return a.exec();

#endif
}
