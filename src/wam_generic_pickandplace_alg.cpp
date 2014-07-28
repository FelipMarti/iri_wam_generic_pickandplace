#include "wam_generic_pickandplace_alg.h"

WamGenericPickandplaceAlgorithm::WamGenericPickandplaceAlgorithm(void)
{
	pthread_mutex_init(&this->access_, NULL);
}

WamGenericPickandplaceAlgorithm::~WamGenericPickandplaceAlgorithm(void)
{
	pthread_mutex_destroy(&this->access_);
}

void WamGenericPickandplaceAlgorithm::config_update(Config & new_cfg,
						    uint32_t level)
{
	this->lock();

	// save the current configuration
	this->config_ = new_cfg;

	this->unlock();
}

// WamGenericPickandplaceAlgorithm Public API
