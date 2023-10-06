#!/usr/bin/env python3
import elsie
from elsie.ext import unordered_list
from elsie.boxtree.box import Box

slides = elsie.SlideDeck(width=1920, height=1080)

slides.update_style("default", elsie.TextStyle(font="Lato", align="left", size=64))
slides.update_style("code", elsie.TextStyle(size=38))
slides.set_style("link", elsie.TextStyle(color="blue"))


def logo_header_slide(parent: Box, title: str):
    # parent.box(x=1570, y=40).image("picknik_logo.png")
    parent.sbox(name="header", x=0, height=140).fbox(p_left=20).text(
        title, elsie.TextStyle(bold=True)
    )
    return parent.fbox(name="content", p_left=20, p_right=20)


def image_slide(parent: Box, title: str, image_path: str):
    content = logo_header_slide(parent, title)
    content = content.fbox(horizontal=True, p_top=20, p_bottom=20)
    text_area = content.fbox(name="text_area", width="50%")
    content.sbox(name="image", width="fill").image(image_path)
    return text_area


def section_title_slide(parent: Box, title: str, subtitle: str):
    content = logo_header_slide(parent, "")
    content.sbox().text(title, elsie.TextStyle(align="right", size=240, bold=True))
    content.box().text(subtitle)


def code_slide(parent: Box, title: str, language: str, code: str):
    content = logo_header_slide(parent, title)
    code_bg = "#F6F8FA"
    box = content.box(y=0, width="100%", height="100%", p_bottom=20)
    box.rect(bg_color=code_bg, rx=20, ry=20)
    box.overlay().box(x=0, y=0, p_left=20, p_right=20, p_top=20, p_bottom=20).code(
        language, code
    )

@slides.slide(debug_boxes=False)
def title(slide):
    content = logo_header_slide(slide, "")
    content.box(width="fill").text(
        "spanny", elsie.TextStyle(size=160, bold=True)
    )
    content.box(width="fill").text(
        "abusing mdspan is within arms reach", elsie.TextStyle(size=72, bold=True, italic=True))
    content.box(width="fill", p_top=10).text("October 4, 2023")
    content.box(width="fill", p_top=180).text("Griswald Brooks")
    content.box(width="fill").text("Senior Software Engineer\ngriswald.brooks@picknik.ai")

@slides.slide(debug_boxes=False)
def normal_usage(slide):
    code_slide(
        slide,
        "mdspan",
        "C++",
        """
// Allocate the map
std::array map_storage{...};
// Create a 2D view of the map
auto occupancy_map = std::mdspan(map_storage.data(), 384, 384);
// Update the map 
for (std::size_t i = 0; i != occupancy_map.extent(0); i++) {
    for (std::size_t j = 0; j != occupancy_map.extent(1); j++) {
        occupancy_map[i, j] = get_occupancy_from_laser_scan(...)
    }
}
// TODO: Add turtlebot map with attribution
""",
    )
    slide.set_style("footer", elsie.TextStyle(color="black", size=32, align="right"))
    footer = slide.box(width="fill", height="5%", horizontal=True)
    f1 = footer.box(width="fill", height="fill")
    f1.text("github.com/griswaldbrooks/spanny", "footer")


@slides.slide(debug_boxes=False)
def policy_injection(slide):
    code_slide(
        slide,
        "Policy Injection",
        "C++",
        """
template <
  class T,
  class Extents,
  class LayoutPolicy = std::layout_right,
  class Accessor = std::default_accessor
>
class mdspan;
""",
    )
    slide.set_style("footer", elsie.TextStyle(color="black", size=32, align="right"))
    footer = slide.box(width="fill", height="5%", horizontal=True)
    f1 = footer.box(width="fill", height="fill")
    f1.text("github.com/griswaldbrooks/spanny", "footer")


@slides.slide(debug_boxes=False)
def bin_accessor(slide):
    code_slide(
        slide,
        "Bin Accessor Policy",
        "C++",
        """
template <int nbins>
struct bin_checker {
  using element_type = tl::expected<bin_state, std::string>;
  using reference = tl::expected<bin_state, std::string> const&;
  using data_handle_type = robot_arm*;

  inline static element_type const no_bin = tl::make_unexpected("Invalid bin");
  mutable element_type recent_ = tl::make_unexpected("Bin has not been accessed");

  reference access(data_handle_type arm, std::ptrdiff_t offset) const {
    // Return an error if the offset is out of bounds
    if (offset < 0 || offset >= nbins) return no_bin;
    // Set the bin state in a member of the accessor policy
    recent_ = arm->is_bin_occupied(static_cast<int>(offset));
    // This is because we return a reference
    return recent_;
  }
};
""",
    )
    slide.set_style("footer", elsie.TextStyle(color="black", size=32, align="right"))
    footer = slide.box(width="fill", height="5%", horizontal=True)
    f1 = footer.box(width="fill", height="fill")
    f1.text("github.com/griswaldbrooks/spanny", "footer")


@slides.slide(debug_boxes=False)
def sync_implementation(slide):
    code_slide(
        slide,
        "Getting Started",
        "C++",
        """
using ext_t = stdex::extents<uint32_t, 2, 3>;
using acc_t = bin_checker<6>;
using bin_view = stdex::mdspan<bin_state, ext_t, stdex::layout_right, acc_t>;

int main(int, char **) {
  auto arm = robot_arm{"/dev/ttyACM0", 9600};
  auto bins = bin_view(&arm, {}, bin_checker<6>{});
  while(true) {
    for (std::size_t i = 0; i != bins.extent(0); ++i) {
      for (std::size_t j = 0; j != bins.extent(1); ++j) {
        std::cout << "Bin " << i << ", " << j << " is ";
        bins(i, j).and_then(print_state).or_else(shrug);
        std::cout << "\\n";
      }
    }
    std::cout << "====================\\n";
  }
  return 0;
}
""",
    )
    slide.set_style("footer", elsie.TextStyle(color="black", size=32, align="right"))
    footer = slide.box(width="fill", height="5%", horizontal=True)
    f1 = footer.box(width="fill", height="fill")
    f1.text("github.com/griswaldbrooks/spanny", "footer")


@slides.slide(debug_boxes=False)
def demo_time(slide):
    content = logo_header_slide(slide, "")
    content.box(width="fill").text(
        "Demo Time", elsie.TextStyle(size=160, bold=True)
    )
    
    slide.set_style("footer", elsie.TextStyle(color="black", size=32, align="right"))
    footer = slide.box(width="fill", height="5%", horizontal=True)
    f1 = footer.box(width="fill", height="fill")
    f1.text("github.com/griswaldbrooks/spanny", "footer")
    
@slides.slide(debug_boxes=False)
def thank_you(slide):
    content = logo_header_slide(slide, "")
    content.fbox().text(
        "Thank you!\n~link{github.com/griswaldbrooks/spanny}\n",
        elsie.TextStyle(align="middle"),
    )
    content.sbox(p_bottom=20).text(
        "Slides generated using Elsie\n",
        elsie.TextStyle(align="right", size=32),
    )


slides.render("spanny_talk.pdf")
