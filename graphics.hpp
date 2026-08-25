#ifndef PROJECT_GRAPHICS_HPP
#define PROJECT_GRAPHICS_HPP

#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Network.hpp>
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
  sf::CircleShape dot{10.0f};

  // Private functions

  void initVariables();
  void initWindows();

 public:
  // Constructors / Destructors
  Sim();
  virtual ~Sim();

  // Accessors
  bool running() const;

  // Methods
  void update();
  void render();
  void display_chart(std::vector<point<float>> LinearMomentum,
                     std::vector<float> AngularMomentum);
  void pollEvents();
  void initBodies(System& sys);
};
}  // namespace project

#endif
