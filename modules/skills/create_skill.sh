SKILL_NAME=$1
SNAKE_CASE_NAME=$(echo $SKILL_NAME | sed -r 's/([A-Z])/_\L\1/g' | sed 's/^_//')    
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BOOK_NAME=bod_skill_book
ENUM_NAME=BodSkillNames
HEADER_FILE=$SCRIPT_DIR/include/skill_books/$BOOK_NAME/$SNAKE_CASE_NAME.hpp

#check if skill exists
if test -f "$HEADER_FILE"; then
    echo "Skill allready exists..."
    exit
fi


#write header content
printf "#pragma once

#include \"skills/skill_builder.hpp\"
#include \"local_planner/skills/skill_util.hpp\"

namespace luhsoccer::skills
{

SKILL_DEFS

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
class ${SKILL_NAME}Build : public SkillBuilder
{
   public:
    ${SKILL_NAME}Build();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills" > $HEADER_FILE


# write source file content
SOURCE_FILE=$SCRIPT_DIR/src/$BOOK_NAME/$SNAKE_CASE_NAME.cpp

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

void ${SKILL_NAME}Build::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step

    // end of skill
}
}  // namespace luhsoccer::skills" > $SOURCE_FILE

# add to cmake list
CMAKELIST=$SCRIPT_DIR/CMakeLists.txt

INSERT_TEXT="\    src/${BOOK_NAME}/${SNAKE_CASE_NAME}.cpp\n    include/skill_books/${BOOK_NAME}/${SNAKE_CASE_NAME}.hpp"

sed -i "/^    # end of skills: DO NOT delete this line/i$INSERT_TEXT" $CMAKELIST 

#add to skillbook
SKILL_BOOK_HEADER=$SCRIPT_DIR/include/skill_books/$BOOK_NAME.hpp
SKILL_BOOK_FILE=$SCRIPT_DIR/src/$BOOK_NAME/$BOOK_NAME.cpp

sed -i "/^\/\/ end of includes DO NOT MODIFY THIS LINE/i #include \"skill_books/${BOOK_NAME}/${SNAKE_CASE_NAME}.hpp\"" $SKILL_BOOK_FILE 

sed -i "/^};  \/\/ DO NOT MODIFY THIS LINE/i \    ${SNAKE_CASE_NAME^^}," $SKILL_BOOK_HEADER 

sed -i "/^          \/\/ do not delete this line either/i \          {$ENUM_NAME::${SNAKE_CASE_NAME^^},${SKILL_NAME}Build().build(cs)}," $SKILL_BOOK_FILE 
echo "Created template for ${SKILL_NAME} skill."

#add to pybind
PYBIND_FILE=$SCRIPT_DIR/../python_bindings/src/bindings/skills.cpp

sed -i "/^}  \/\/ DO NOT EDIT THIS LINE/i \    instance.value(\"${SKILL_NAME}\", $ENUM_NAME::${SNAKE_CASE_NAME^^});" $PYBIND_FILE 