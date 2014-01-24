# collect data
file(GLOB DATA_FILES *)
file(COPY ${DATA_FILES} DESTINATION .)
