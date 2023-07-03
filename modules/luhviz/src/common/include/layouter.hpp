#include <vector>
namespace luhsoccer::luhviz {

class Layouter {
   public:
    Layouter(float width_in_percentage, float relative_height, float relative_x, float relative_y, float padding,
             const Layouter& parent)
        : relative_width(width_in_percentage),
          relative_height(relative_height),
          relative_x(relative_x),
          relative_y(relative_y),
          padding(padding),
          parent(parent){};

    float getWidth() { return this->width; }

    float getHeight() { return this->height; }

    float getX() { return this->x; }

    float getY() { return this->y; }

    float getPadding() { return this->padding; }

    Layouter getParent() { return this->parent; }

    std::vector<Layouter> getChilds() { return this->childs; }

    void addChild(Layouter& c) { this->childs.emplace_back(c); }

    void recalculateSizesRecursively(float width, float height, float x, float y) {
        this->width = width * relative_width;
        this->height = height * relative_height;
        this->x = x + relative_x;
        this->y = y + relative_y;

        for (Layouter& child : this->childs) {
            child.recalculateSizesRecursively(this->width, this->height, this->x + this->padding,
                                              this->y + this->padding);
        }
    }

    void layout() {
        for (Layouter& child : this->childs) {
            child.layout();
        }
    }

   private:
    float width;
    float relative_width;
    float height;
    float relative_height;
    float x;
    float relative_x;
    float y;
    float relative_y;
    float padding;

    const Layouter& parent;
    std::vector<Layouter> childs;
};
}  // namespace luhsoccer::luhviz