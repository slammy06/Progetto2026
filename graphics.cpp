#include "graphics.hpp"

#include <algorithm>
#include <cmath>

using namespace project;
// Private functions
void Sim::initVariables() {
  this->SimWindow = nullptr;
  this->ChartWindow = nullptr;
  this->chartFont.loadFromFile("DejaVuSans.ttf");
}
void Sim::initWindows() {
  this->VideoMode1.height = 1000;
  this->VideoMode1.width = 1000;
  this->VideoMode2.height = 1000;
  this->VideoMode2.width = 1200;

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
bool Sim::graphing() const { return this->ChartWindow->isOpen(); }
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
                        std::vector<float> AngMom,
                        std::vector<float> TotalEnergy) {
  this->dot.setOrigin({2.f, 2.f});
  // Setting scales
  const std::size_t samples =
      std::min({LinMom.size(), AngMom.size(), TotalEnergy.size()});
  if (samples == 0) return;

  const float panelHeight = static_cast<float>(this->VideoMode2.height) / 4.0f;
  const float chartWidth = static_cast<float>(this->VideoMode2.width);
  const float chartLeft = 55.0f;
  const float plotWidth = chartWidth - chartLeft - 10.0f;

  std::vector<float> linearX(samples);
  std::vector<float> linearY(samples);
  for (std::size_t i = 0; i < samples; ++i) {
    linearX[i] = LinMom[i].x;
    linearY[i] = LinMom[i].y;
  }

  const auto maximumAbsolute = [](const std::vector<float>& values) {
    float result = 0.0f;
    for (float value : values) {
      result = std::max(result, std::abs(value));
    }
    return result;
  };
    const auto minimumAbsolute = [](const std::vector<double>& values) {
    double result = 0.;
    for (double value : values) {
      result = -std::max(result, std::abs(value));
    }
    return result;
  };
  // Enumerating the samples and their maximums + other style settings
  const double scales[] = {maximumAbsolute(AngMom), maximumAbsolute(linearX),
                          
                          maximumAbsolute(TotalEnergy)};
  const std::vector<float>* values[] = {&AngMom, &linearX, &linearY,
                                        &TotalEnergy};
  const sf::Color colors[] = {sf::Color::Blue, sf::Color::Red, sf::Color::Green,
                              sf::Color::Yellow};
  const char* titles[] = {"Angular momentum Lz", "Linear momentum Px",
                          "Linear momentum Py", "Total energy"};

  this->ChartWindow->clear(sf::Color::White);
  // main frame rendering loop
  for (int panel = 0; panel < 4; ++panel) {
    const float baseline = (static_cast<float>(panel) + 0.65f) * panelHeight;
    const float scale =
        scales[panel] == 0.0f ? 0.0f : (panelHeight * 0.45f) / scales[panel];
    this->dot.setFillColor(colors[panel]);

    sf::Vertex axes[] = {
        sf::Vertex({chartLeft, baseline - panelHeight * 0.45f},
                   sf::Color(100, 100, 100)),
        sf::Vertex({chartLeft, baseline + panelHeight * 0.45f},
                   sf::Color(100, 100, 100)),
     sf::Vertex({chartWidth - 10.0f, baseline}, sf::Color(100, 100, 100)),
        sf::Vertex({chartLeft, baseline}, sf::Color(100, 100, 100))};
    this->ChartWindow->draw(axes, 4, sf::Lines);

    sf::Text title(titles[panel], this->chartFont, 18);
    title.setFillColor(sf::Color::Black);
    title.setPosition({chartLeft, baseline - panelHeight * 0.45f - 24.0f});
    this->ChartWindow->draw(title);

    sf::Text yLabel("value", this->chartFont, 12);
    yLabel.setFillColor(sf::Color(80, 80, 80));
    yLabel.setPosition({5.0f, baseline - panelHeight * 0.45f});
    this->ChartWindow->draw(yLabel);

    sf::Text xLabel("Time samples", this->chartFont, 12);
    xLabel.setFillColor(sf::Color(80, 80, 80));
    xLabel.setPosition({chartWidth - 145.0f, baseline + 5.0f});
    this->ChartWindow->draw(xLabel);

    for (std::size_t i = 0; i < samples; ++i) {
      const float x = samples == 1
                          ? chartLeft
                          : chartLeft + static_cast<float>(i) * plotWidth /
                                            static_cast<float>(samples - 1);
      const float y = baseline - (*values[panel])[i] * scale;
      this->dot.setPosition({x, y});
      this->ChartWindow->draw(dot);
    }
  }

  this->ChartWindow->display();
}