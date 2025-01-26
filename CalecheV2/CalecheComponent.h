#ifndef COMPONENT_H
#define COMPONENT_H

class CalecheComponent
{
	public:
		virtual void setup() {};   // Optional setup
		virtual void start() {};   // Optional start
		virtual void shutdown() {}; // Optional shutdown
		virtual void update() = 0;  // Mandatory update
		virtual ~CalecheComponent() {};   // Virtual destructor
};

#endif // COMPONENT_H
