# Board Makefile fragment for bossbrain_f427
# Defines include path and preprocessor macros for board selection

# Specify board-specific include directory for header files
BOARDINC = $(PROJECT_DIR)/config/boards/bossbrain_f427

# If you have a board-specific configuration source file, uncomment and use:
BOARDCPPSRC = $(PROJECT_DIR)/config/boards/bossbrain_f427/board_configuration.cpp

# Board identifier macros
DDEFS += -DSHORT_BOARD_NAME=bossbrain_f427
DDEFS += -DFIRMWARE_ID=\"bossbrain_f427\"

# Set default engine type (adjust to match your intended board default)
#DDEFS += -DDEFAULT_ENGINE_TYPE=MINIMAL_PINS