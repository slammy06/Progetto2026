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
  this->VideoMode1.width = 900;
  this->VideoMode2.height = 1000;
  this->VideoMode2.width = 1000;

  this->SimWindow =
      new sf::RenderWindow(this->VideoMode1, "N-Body Simulation",
                           sf::Style::Titlebar | sf::Style::Close);
  this->ChartWindow =
      new sf::RenderWindow(this->VideoMode2, "Momentum Chart",
                           sf::Style::Titlebar | sf::Style::Close);
  this->SimWindow->setFramerateLimit(60);
  this->SimWindow->setPosition({0, 0});
  this->ChartWindow->setFramerateLimit(60);
  this->ChartWindow->setPosition({900, 0});
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
  const float centerX = 450.0f;
  const float centerY = 500.0f;
  const float scale = 400.0f;
  for (long unsigned int i = 0; i < sys.get_bodies().size(); ++i) {
    this->bodies[i].setFillColor(sf::Color::White);
    const float rad = static_cast<float>(sys.get_bodies()[i].radius * scale);
    this->bodies[i].setRadius(rad);
    this->bodies[i].setOrigin(
        {rad, rad});
        const float x = static_cast<float>(centerX + sys.get_bodies()[i].pos.x * scale);
        const float y = static_cast<float>(centerY + sys.get_bodies()[i].pos.y * scale);
    this->bodies[i].setPosition({x, y});

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
void Sim::display_chart(std::vector<point<double>> LinMom,
                        std::vector<double> AngMom,
                        std::vector<double> TotalEnergy) {
  this->dot.setOrigin({2.f, 2.f});
  // Setting scales
  const std::size_t samples =
      std::min({LinMom.size(), AngMom.size(), TotalEnergy.size()});
  if (samples == 0) return;

  const float panelHeight = static_cast<float>(this->VideoMode2.height) / 4.0f;
  const float chartWidth = static_cast<float>(this->VideoMode2.width);
  const float chartLeft = 55.0f;
  const float plotWidth = chartWidth - chartLeft - 10.0f;

  std::vector<double> linearX(samples);
  for (std::size_t i = 0; i < samples; ++i) {
    linearX[i] = LinMom[i].norm();
  }

  const auto maximumAbsolute = [](const std::vector<double>& values) {
    double result = 0.;
    for (double value : values) {
      result = std::max(result, std::abs(value));
    }
    return result;
  };
    const auto minimumAbsolute = [](const std::vector<double>& values) {
    double result = 0.;
    for (double value : values) {
      result = std::min(result, std::abs(value));
    }
    return result;
  };
  // Enumerating the samples and their maximums + other style settings
  const double scales[] = {maximumAbsolute(AngMom), maximumAbsolute(linearX),
                          
                          maximumAbsolute(TotalEnergy)};
  const std::vector<double>* values[] = {&AngMom, &linearX,
                                        &TotalEnergy};
  const sf::Color colors[] = {sf::Color::Blue, sf::Color::Red,
                              sf::Color::Yellow};
  const char* titles[] = {"Angular momentum Lz", "Linear momentum", "Total energy"};

  this->ChartWindow->clear(sf::Color::White);
  // main frame rendering loop
  for (int panel = 0; panel < 3; ++panel) {
    const float baseline = (static_cast<float>(panel) + 0.65f) * panelHeight;
    const double scale =
        scales[panel] == 0.0f ? 0.0f : (panelHeight * 0.45f) / scales[panel];
    this->dot.setFillColor(colors[panel]);

    sf::Vertex axes[] = {
        sf::Vertex({chartLeft, baseline - panelHeight * 0.45f},
                   sf::Color(100, 100, 100)),
        sf::Vertex({chartLeft, baseline + panelHeight * 0.45f},
                   sf::Color(100, 100, 100)),
        sf::Vertex({chartLeft, baseline}, sf::Color(100, 100, 100))};
    this->ChartWindow->draw(axes, 3, sf::Lines);

    sf::Text title(titles[panel], this->chartFont, 18);
    title.setFillColor(sf::Color::Black);
    title.setPosition({chartLeft, baseline - panelHeight * 0.45f - 24.0f});
    this->ChartWindow->draw(title);

    sf::Text xLabel("Time samples", this->chartFont, 12);
    xLabel.setFillColor(sf::Color(80, 80, 80));
    xLabel.setPosition({chartWidth - 145.0f, baseline + 5.0f});
    this->ChartWindow->draw(xLabel);

    for (std::size_t i = 0; i < samples; ++i) {
      const float x = samples == 1
                          ? chartLeft
                          : chartLeft + static_cast<float>(i) * plotWidth /
                                            static_cast<float>(samples - 1);
      const float y = static_cast<float>(baseline - (*values[panel])[i] * scale);
      this->dot.setPosition({x, y});
      this->ChartWindow->draw(dot);
    }
    sf::Text maxLabel(
    std::to_string(maximumAbsolute(*values[panel])),
    this->chartFont, 12
);

maxLabel.setFillColor(sf::Color::Black);
maxLabel.setPosition({
    5.0f,
    baseline - panelHeight * 0.45f
});

this->ChartWindow->draw(maxLabel);
sf::Text minLabel(
    std::to_string(minimumAbsolute(*values[panel])),
    this->chartFont, 12
);

minLabel.setFillColor(sf::Color::Black);

minLabel.setPosition({
    5.0f,
    baseline + panelHeight * 0.45f - 15.0f
});

this->ChartWindow->draw(minLabel);
  }

  this->ChartWindow->display();
}