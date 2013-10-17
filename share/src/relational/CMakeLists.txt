set(SOURCES     ${SOURCES}
conceptLearner.cpp
decisionMakingModule.cpp
experiments_fixedContexts.cpp
explore.cpp
learn.cpp
learn_searchOperators.cpp
literals.cpp
plan.cpp
ppddl.cpp
pradaBackwards.cpp
prada.cpp
reason.cpp
reason_rules.cpp
relevanceGrounding.cpp
robotManipulationInterface.cpp
robotManipulationSimulator.cpp
robotManipulationSymbols.cpp
ruleLearner_ground.cpp
rules.cpp
symbolGrounding.cpp
symbols.cpp
utilTL.cpp
)

add_library(relational SHARED ${SOURCES})

target_link_libraries(relational LINK_PRIVATE Core Ors)



