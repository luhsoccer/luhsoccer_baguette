SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
help() {
    echo "Usage: create_skill.sh <command> <arguments>"
    echo "Commands:"
    echo "  create <skill_book> <skill_name>   - Create a new skill"
    echo "  remove <skill_book> <skill_name>   - Remove an existing skill"
    echo "  rename <skill_book> <old_name> <new_name>   - Rename an existing skill"
    echo "  move <skill_book> <skill_name> <new_skill_book>   - Move a skill to a different skill book"
    echo "  help                               - Show this help page"
}

get_enum_name(){
    case $1 in
        "bod")
            echo "BodSkillNames"
            ;;
        "game")
            echo "GameSkillNames"
            ;;
        "test")
            echo "TestSkillNames"
            ;;
        *)
            echo "Unsupported skill book, Exiting..."
            help
            exit
            ;;
    esac
}

get_book_name(){
    case $1 in
        "bod")
            echo "bod_skill_book"
            ;;
        "game")
            echo "game_skill_book"
            ;;
        "test")
            echo "test_skill_book"
            ;;
        *)
            echo "Unsupported skill book, Exiting..."
            help
            exit
            ;;
    esac

}

check_skill_name(){
    if [ -z "$1" ]; then
        echo "No skill name provided. Exiting..."
        echo "Exiting..."
        help
        exit
    fi


    if [[ $1 == *"_"* ]]; then
    echo "_ not allowed in skill name. Use CamelCase instead. e.g. MySkillName. Exiting..."
    help
    exit
    fi

    if [[ $1 =~ ^[a-z] ]]; then
        echo "Skill name must start with an uppercase letter. Use CamelCase instead. e.g. MySkillName. Exiting..."
        help
        exit
    fi
}

add_skill_to_skill_book(){
    local BOOK_NAME=$(get_book_name $1)
    local ENUM_NAME=$(get_enum_name $1)
    local SNAKE_CASE_NAME=$(echo $2 | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//')    
    local SKILL_NAME=$2
    local SKILL_BOOK_HEADER=$SCRIPT_DIR/include/skill_books/$BOOK_NAME.hpp
    local SKILL_BOOK_FILE=$SCRIPT_DIR/src/$BOOK_NAME/$BOOK_NAME.cpp

    sed -i "/^\/\/ end of includes DO NOT MODIFY THIS LINE/i #include \"skill_books/${BOOK_NAME}/${SNAKE_CASE_NAME}.hpp\"" $SKILL_BOOK_FILE 

    sed -i "/^};  \/\/ DO NOT MODIFY THIS LINE/i \    ${SNAKE_CASE_NAME^^}," $SKILL_BOOK_HEADER 

    sed -i "/^          \/\/ do not delete this line either/i \          {$ENUM_NAME::${SNAKE_CASE_NAME^^}, ${SKILL_NAME}Build().build(cs)}," $SKILL_BOOK_FILE 
}

add_skill_to_cmake_list(){
    local BOOK_NAME=$(get_book_name $1)
    local SNAKE_CASE_NAME=$(echo $2 | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//')    
    # add to cmake list
    local CMAKELIST=$SCRIPT_DIR/CMakeLists.txt

    INSERT_TEXT="\    src/${BOOK_NAME}/${SNAKE_CASE_NAME}.cpp\n    include/skill_books/${BOOK_NAME}/${SNAKE_CASE_NAME}.hpp"

    sed -i "/^    # end of skills: DO NOT delete this line/i$INSERT_TEXT" $CMAKELIST 
}

remove_skill_from_skill_book(){
    local BOOK_NAME=$(get_book_name $1)
    local ENUM_NAME=$(get_enum_name $1)
    local SNAKE_CASE_NAME=$(echo $2 | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//')    
    local SKILL_NAME=$2
    local SKILL_BOOK_HEADER=$SCRIPT_DIR/include/skill_books/$BOOK_NAME.hpp
    local SKILL_BOOK_FILE=$SCRIPT_DIR/src/$BOOK_NAME/$BOOK_NAME.cpp

    sed -i "/^#include \"skill_books\/${BOOK_NAME}\/${SNAKE_CASE_NAME}.hpp\"/d " $SKILL_BOOK_FILE 

    sed -i "/    ${SNAKE_CASE_NAME^^},/d " $SKILL_BOOK_HEADER 

    sed -i "/          {$ENUM_NAME::${SNAKE_CASE_NAME^^}, ${SKILL_NAME}Build().build(cs)},/d " $SKILL_BOOK_FILE 
}

remove_skill_from_cmake_list(){
    local BOOK_NAME=$(get_book_name $1)
    local SNAKE_CASE_NAME=$(echo $2 | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//')    
    # add to cmake list
    local CMAKELIST=$SCRIPT_DIR/CMakeLists.txt
    sed -i "/^    src\/${BOOK_NAME}\/${SNAKE_CASE_NAME}.cpp/d" $CMAKELIST 
    sed -i "/^    include\/skill_books\/${BOOK_NAME}\/${SNAKE_CASE_NAME}.hpp/d" $CMAKELIST 
}

create() {
    local SKILL_NAME=$2
    local SKILL_BOOK_NAME=$1

    # check params
    if [ -z "$SKILL_BOOK_NAME" ]; then
        echo "No skill book name provided. Usage: create_skill.sh create {\"bod\",\"game\",\"test\"}  skillName"
        echo "Exiting..."
        help
        exit
    fi

    if [ -z "$SKILL_NAME" ]; then
        echo "No skill name provided. Usage: create_skill.sh create {\"bod\",\"game\",\"test\"} skillName"
        echo "Exiting..."
        help
        exit
    fi

    check_skill_name $SKILL_NAME

    local SNAKE_CASE_NAME=$(echo $SKILL_NAME | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//')    
    local BOOK_NAME=$(get_book_name $SKILL_BOOK_NAME)
    local ENUM_NAME=$(get_enum_name $SKILL_BOOK_NAME)

    # create header file
    local HEADER_FILE=$SCRIPT_DIR/include/skill_books/$BOOK_NAME/$SNAKE_CASE_NAME.hpp

    #check if skill exists
    if test -f "$HEADER_FILE"; then
        echo "Skill allready exists..."
        exit
    fi


    #write header content
    printf "#pragma once

#include \"skills/skill_builder.hpp\"
#include \"robot_control/components/component_util.hpp\"

namespace luhsoccer::skills {

using namespace robot_control;
// NOLINTNEXTLINE(readability-identifier-naming)
using TD_Pos = ComponentPosition::TaskDataType;

/**
 * @brief Short description
 *
 * Detailed description
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class ${SKILL_NAME}Build : public SkillBuilder {
   public:
    ${SKILL_NAME}Build();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills" > $HEADER_FILE


    # write source file content
    local SOURCE_FILE=$SCRIPT_DIR/src/$BOOK_NAME/$SNAKE_CASE_NAME.cpp

    printf "#include \"skill_books/${BOOK_NAME}/${SNAKE_CASE_NAME}.hpp\"
// include components here

namespace luhsoccer::skills {

${SKILL_NAME}Build::${SKILL_NAME}Build()
    : SkillBuilder(\"${SKILL_NAME}\",  //
                {},           //
                {},           //
                {},           //
                {},           //
                {},           //
                {}){};

void ${SKILL_NAME}Build::buildImpl(const config_provider::ConfigStore& /*cs*/) {
    // Add skill definition here. Use addStep to add a step

    // end of skill
}
}  // namespace luhsoccer::skills" > $SOURCE_FILE

   
    add_skill_to_cmake_list $SKILL_BOOK_NAME $SKILL_NAME
    add_skill_to_skill_book $SKILL_BOOK_NAME $SKILL_NAME
    echo "Created template for ${SKILL_NAME} skill."
}


remove() {
    local SKILL_NAME=$2
    local SKILL_BOOK_NAME=$1
    check_skill_name $SKILL_NAME
    
    local BOOK_NAME=$(get_book_name $SKILL_BOOK_NAME)
    local SNAKE_CASE_NAME=$(echo $SKILL_NAME | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//') 
    local HEADER_FILE=$SCRIPT_DIR/include/skill_books/$BOOK_NAME/$SNAKE_CASE_NAME.hpp

    if ! test -f "$HEADER_FILE"; then
        echo "Skill does not exist. Exiting..."
        exit
    fi

    rm $HEADER_FILE
    rm $SCRIPT_DIR/src/$BOOK_NAME/$SNAKE_CASE_NAME.cpp

    remove_skill_from_skill_book $SKILL_BOOK_NAME $SKILL_NAME
    remove_skill_from_cmake_list $SKILL_BOOK_NAME $SKILL_NAME

    echo "Removed $SKILL_NAME from $SKILL_BOOK_NAME skill book."
}

rename() {
    local SKILL_NAME=$2
    local NEW_SKILL_NAME=$3
    local SKILL_BOOK_NAME=$1

    check_skill_name $SKILL_NAME
    check_skill_name $NEW_SKILL_NAME

    local BOOK_NAME=$(get_book_name $SKILL_BOOK_NAME)
    local SNAKE_CASE_NAME=$(echo $SKILL_NAME | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//') 
    local HEADER_FILE=$SCRIPT_DIR/include/skill_books/$BOOK_NAME/$SNAKE_CASE_NAME.hpp

    if ! test -f "$HEADER_FILE"; then
        echo "Skill does not exist. Exiting..."
        exit
    fi

    remove_skill_from_skill_book $SKILL_BOOK_NAME $SKILL_NAME
    remove_skill_from_cmake_list $SKILL_BOOK_NAME $SKILL_NAME

    add_skill_to_skill_book $SKILL_BOOK_NAME $NEW_SKILL_NAME
    add_skill_to_cmake_list $SKILL_BOOK_NAME $NEW_SKILL_NAME
    
    local NEW_SNAKE_CASE_NAME=$(echo $NEW_SKILL_NAME | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//') 
    local NEW_HEADER_FILE=$SCRIPT_DIR/include/skill_books/$BOOK_NAME/$NEW_SNAKE_CASE_NAME.hpp

    local SOURCE_FILE=$SCRIPT_DIR/src/$BOOK_NAME/$SNAKE_CASE_NAME.cpp
    local NEW_SOURCE_FILE=$SCRIPT_DIR/src/$BOOK_NAME/$NEW_SNAKE_CASE_NAME.cpp

    mv $HEADER_FILE $NEW_HEADER_FILE
    sed -i "s/${SKILL_NAME}/${NEW_SKILL_NAME}/g" $NEW_HEADER_FILE
    
    mv $SOURCE_FILE $NEW_SOURCE_FILE
    sed -i "s/${SKILL_NAME}/${NEW_SKILL_NAME}/g" $NEW_SOURCE_FILE
    sed -i "s/${SNAKE_CASE_NAME}/${NEW_SNAKE_CASE_NAME}/g" $NEW_SOURCE_FILE
    
}

move () {
    local SKILL_NAME=$2
    local SKILL_BOOK_NAME=$1
    local NEW_SKILL_BOOK_NAME=$3

    check_skill_name $SKILL_NAME

    local BOOK_NAME=$(get_book_name $SKILL_BOOK_NAME)
    local SNAKE_CASE_NAME=$(echo $SKILL_NAME | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//') 
    local HEADER_FILE=$SCRIPT_DIR/include/skill_books/$BOOK_NAME/$SNAKE_CASE_NAME.hpp

    if ! test -f "$HEADER_FILE"; then
        echo "Skill does not exist. Exiting..."
        exit
    fi

    remove_skill_from_skill_book $SKILL_BOOK_NAME $SKILL_NAME
    remove_skill_from_cmake_list $SKILL_BOOK_NAME $SKILL_NAME

    add_skill_to_skill_book $NEW_SKILL_BOOK_NAME $SKILL_NAME
    add_skill_to_cmake_list $NEW_SKILL_BOOK_NAME $SKILL_NAME
    
    local NEW_BOOK_NAME=$(get_book_name $NEW_SKILL_BOOK_NAME)
    local NEW_HEADER_FILE=$SCRIPT_DIR/include/skill_books/$NEW_BOOK_NAME/$SNAKE_CASE_NAME.hpp

    if test -f "$NEW_HEADER_FILE"; then
        echo "Skill already exist in new skill book. Exiting..."
        exit
    fi

    local SOURCE_FILE=$SCRIPT_DIR/src/$BOOK_NAME/$SNAKE_CASE_NAME.cpp
    local NEW_SOURCE_FILE=$SCRIPT_DIR/src/$NEW_BOOK_NAME/$SNAKE_CASE_NAME.cpp

    mv $HEADER_FILE $NEW_HEADER_FILE
    mv $SOURCE_FILE $NEW_SOURCE_FILE
    sed -i "s/${BOOK_NAME}/${NEW_BOOK_NAME}/g" $NEW_SOURCE_FILE
}



COMMAND=$1
case $COMMAND in
    "create")
        create "$2" "$3";
        ;;

    "remove")
        remove "$2" "$3";
        ;;
    
    "rename")
        rename "$2" "$3" "$4";
        ;;

    "move")
        move "$2" "$3" "$4";
        ;;

    "help")
        help;
        ;;

    *)
        echo "Unknown command. Use: create, move, delete, rename, help. Exiting..."
        exit
        ;;
esac
