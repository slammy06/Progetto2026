#include "graphics.hpp"

#include "main.hpp"

namespace project {
// Private functions
void Sim::initVariables() { this->window = nullptr; }
void Sim::initWindow() {
  this->VideoMode.height = 800;
  this->VideoMode.width = 800;

  this->window = new sf::RenderWindow(this->VideoMode, "N-Body Simulation",
                                      sf::Style::Titlebar | sf::Style::Close);

  this->window->setFramerateLimit(60);
}

// Constructors / Destructors

Sim::Sim() {
  this->initVariables();
  this->initWindow();
}
Sim::~Sim() { delete this->window; }

// Accessors

bool Sim::running() const { return this->window->isOpen(); }

// Methods

void Sim::pollEvents() {
  while (this->window->pollEvent(this->ev)) {
    switch (this->ev.type) {
      case sf::Event::Closed:
        this->window->close();
        break;
      case sf::Event::KeyPressed:
        if (this->ev.key.code == sf::Keyboard::Escape) this->window->close();
        break;
    }
  }
}

void Sim::initBodies(project::System& sys) {
  for (long unsigned int i = 0; i < sys.get_bodies().size(); ++i) {
    this->bodies[i].setFillColor(sf::Color::White);
    this->bodies[i].setOrigin(
        {sys.get_bodies()[i].radius, sys.get_bodies()[i].radius});
    this->bodies[i].setPosition(
        {sys.get_bodies()[i].pos.x, sys.get_bodies()[i].pos.y});
    this->bodies[i].setRadius(sys.get_bodies()[i].radius);
  }
}

void Sim::update() { this->pollEvents(); }

void Sim::render() {
  this->window->clear();

  // Draw Simulation frame
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    this->window->draw(bodies[i]);
  }

  // display frame in window

  this->window->display();
}
}  // namespace project