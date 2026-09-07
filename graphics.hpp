#ifndef PROJECT_GRAPHICS_HPP
#define PROJECT_GRAPHICS_HPP

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "main.hpp"
namespace project {
/*
Class that defines the visuals of the simulation
*/

class Sim {
 private:
  // Variables
  // Window
  sf::RenderWindow* ChartWindow;
  sf::RenderWindow* SimWindow;
  sf::Event ev;
  sf::VideoMode VideoMode1;
  sf::VideoMode VideoMode2;

  // Objects
  std::vector<sf::CircleShape> bodies;
  sf::CircleShape dot{2.0f};
  sf::Font chartFont;

  void check_invariant() const;

  // Private functions

  void initVariables();
  void initWindows();

 public:
  // Constructors / Destructors
  Sim();
  virtual ~Sim();

  // Accessors
  bool running() const;
  bool graphing() const;

  // Methods
  void update();
  void render();
  void display_chart(const std::vector<point<double>>& LinearMomentum,
                     const std::vector<double>& AngularMomentum,
                     const std::vector<double>& TotalEnergy);
  void pollEvents();
  void initBodies(const System& sys);
};
}  // namespace project

#endif
