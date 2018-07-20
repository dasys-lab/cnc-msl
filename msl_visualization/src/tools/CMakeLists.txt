# src/tools

ADD_SUBDIRECTORY( basestation )

ADD_CUSTOM_TARGET( tools DEPENDS
 basestation
)
