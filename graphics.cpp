#include "graphics.hpp"

using namespace project;
// Private functions
void Sim::initVariables() {
  this->SimWindow = nullptr;
  this->ChartWindow = nullptr;
}
void Sim::initWindows() {
  this->VideoMode1.height = 1000;
  this->VideoMode1.width = 1000;
  this->VideoMode2.height = 800;
  this->VideoMode2.width = 1000;

  this->SimWindow =
      new sf::RenderWindow(this->VideoMode1, "N-Body Simulation",
                           sf::Style::Titlebar | sf::Style::Close);
  this->ChartWindow =
      new sf::RenderWindow(this->VideoMode2, "Momentum Chart",
                           sf::Style::Titlebar | sf::Style::Close);
  this->SimWindow->setFramerateLimit(60);
}

// Constructors / Destructors

Sim::Sim() {
  this->initVariables();
  this->initWindows();
}
Sim::~Sim() { delete this->SimWindow; }

// Accessors

bool Sim::running() const { return this->SimWindow->isOpen(); }

// Methods

void Sim::pollEvents() {
  while (this->SimWindow->pollEvent(this->ev)) {
    switch (this->ev.type) {
      case sf::Event::Closed:
        this->SimWindow->close();
        break;
      case sf::Event::KeyPressed:
        if (this->ev.key.code == sf::Keyboard::Escape) this->SimWindow->close();
        break;
      default:
        break;
    }
  }

  while (this->ChartWindow->pollEvent(this->ev)) {
    switch (this->ev.type) {
      case sf::Event::Closed:
        this->ChartWindow->close();
        break;
      case sf::Event::KeyPressed:
        if (this->ev.key.code == sf::Keyboard::Escape)
          this->ChartWindow->close();
        break;
      default:
        break;
    }
  }
}

void Sim::initBodies(project::System& sys) {
  this->bodies.resize(sys.get_bodies().size());
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
  this->SimWindow->clear();

  // Draw Simulation frame
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    this->SimWindow->draw(bodies[i]);
  }

  // display frame in window

  this->SimWindow->display();
}
void Sim::display_chart(std::vector<point<float>> LinMom,
                        std::vector<float> AngMom) {
  this->dot.setOrigin({10.f, 10.f});
  std::vector<float> LinMomX;
  std::vector<float> LinMomY;
  LinMomX.resize(LinMom.size());
  LinMomY.resize(LinMom.size());

  for (long unsigned int i = 0; i < LinMom.size(); ++i) {
    LinMomX.push_back(LinMom[i].x);
    LinMomY.push_back(LinMom[i].y);
  }
  float max = Get_Maximum(std::vector<float>{
      Get_Maximum(LinMomX), Get_Maximum(LinMomY), Get_Minimum(AngMom)});

  float min = Get_Minimum(std::vector<float>{
      Get_Minimum(LinMomX), Get_Minimum(LinMomY), Get_Minimum(AngMom)});
  float offset = 50;
  float scale = (this->VideoMode2.height) / (max + abs(min));
  for (long unsigned int i = 0; i < AngMom.size(); ++i) {
    // Drawing Angular Momentum part of chart in Blue
    this->dot.setFillColor(sf::Color::Blue);

    float x = i * (this->VideoMode2.width) / AngMom.size();
    float y = AngMom[i] * scale + offset;
    this->dot.setPosition({x, y});
    this->ChartWindow->draw(dot);
    // Drawing Linear Momentum on X axis in Red
    this->dot.setFillColor(sf::Color::Red);

    y = LinMomX[i] * scale + 2*offset;
    this->dot.setPosition({x, y});
    this->ChartWindow->draw(dot);
    // Drawing Linear Momentum on Y axis in Green
    this->dot.setFillColor(sf::Color::Green);

    y = LinMomY[i] * scale + 3 * offset;
    this->dot.setPosition({x, y});
    this->ChartWindow->draw(dot);
  }

  this->ChartWindow->display();
}