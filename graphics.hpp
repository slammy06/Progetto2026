#ifndef PROJECT_GRAPHICS_HPP
#define PROJECT_GRAPHICS_HPP

#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Network.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
namespace project{
/*
Class that defines the visuals of the simulation
*/

class Sim {
 private:
  // Variables
  // Window
  sf::RenderWindow* window;
  sf::Event ev;
  sf::VideoMode VideoMode;
  // Objects
  std::vector<sf::CircleShape> bodies;
  // Private functions

  void initVariables();
  void initWindow();
  

 public:
  // Constructors / Destructors
  Sim();
  virtual ~Sim();

  // Accessors
  bool running() const;

  // Methods
  void update();
  void render();
  void pollEvents();
  void initBodies(System& sys);
};
}


#endif

