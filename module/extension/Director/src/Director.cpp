/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2021 NUbots <nubots@nubots.net>
 */
#include "Director.hpp"

#include "extension/Configuration.hpp"

namespace module::extension {

    using ::extension::Configuration;
    using ::extension::behaviour::commands::BehaviourTask;
    using ::extension::behaviour::commands::CausingExpression;
    using ::extension::behaviour::commands::NeedsExpression;
    using ::extension::behaviour::commands::ProviderClassification;
    using ::extension::behaviour::commands::ProviderDone;
    using ::extension::behaviour::commands::ProvideReaction;
    using ::extension::behaviour::commands::WhenExpression;
    using provider::Provider;
    using Unbind = NUClear::dsl::operation::Unbind<ProvideReaction>;

    /**
     * This message gets emitted when a state we are monitoring is updated.
     * When a `When` condition is being waited on a reaction will start monitoring that state.
     * When it updates we check to see if this would now satisfy the conditions and if it does we emit
     */
    struct StateUpdate {
        StateUpdate(const uint64_t& provider_id_, const std::type_index& type_, const int& state_)
            : provider_id(provider_id_), type(type_), state(state_) {}

        /// The reaction id of the Provider which was waiting on this type
        uint64_t provider_id;
        /// The enum type that this enum was listening for
        std::type_index type;
        /// The new value for the enum
        int state;
    };

    void Director::add_provider(const ProvideReaction& provide) {

        auto& group = groups[provide.type];

        // Check if we already inserted this element as a Provider
        if (providers.count(provide.reaction->id) != 0) {
            throw std::runtime_error("You cannot have multiple Provider DSL words in a single on statement.");
        }

        auto provider = std::make_shared<Provider>(provide.classification, provide.type, provide.reaction);
        group.providers.push_back(provider);
        providers.emplace(provide.reaction->id, provider);
    }

    void Director::remove_provider(const uint64_t& id) {

        // If we can find it, erase it
        auto it = providers.find(id);
        if (it != providers.end()) {

            // Get the relevant elements
            auto provider = it->second;
            auto& group   = groups[provider->type];

            // Unbind any when state monitors
            for (auto& when : provider->when) {
                when->handle.unbind();
            }

            // Erase the Provider from this group
            group.providers.erase(std::remove(group.providers.begin(), group.providers.end(), provider),
                                  group.providers.end());

            // Now we need to deal with the cases where this Providers was in use when it was unbound
            if (provider->active) {
                if (group.providers.empty()) {
                    // This is now an error, there are no Providers to service the task
                    log<NUClear::ERROR>("The last Provider for a type was removed while there were still tasks for it");
                }
                else {
                    // Reevaluate the group to see if the loss of this provider changes anything
                    // Since we already removed the provider from the group it won't get reassigned
                    reevaluate_queue(group);
                }
            }

            // If there are no Providers left in this group erase it too
            if (group.providers.empty()) {
                groups.erase(provider->type);
            }
            // Erase from our list of Providers
            providers.erase(id);
        }
    }

    void Director::add_when(const WhenExpression& when) {
        auto it = providers.find(when.reaction->id);
        if (it != providers.end()) {
            auto provider = it->second;

            // Can't add causing to a Start or Stop
            if (provider->classification == ProviderClassification::STOP
                || provider->classification == ProviderClassification::START) {
                throw std::runtime_error("You cannot use the 'When' DSL word with Start or Stop.");
            }

            // The when condition object
            auto w  = std::make_shared<provider::Provider::WhenCondition>(when.type,
                                                                         when.validator,
                                                                         when.validator(when.current()));
            auto id = when.reaction->id;

            // Add a reaction that will listen for changes to this state and notify the director
            w->handle = when.binder(*this, [this, id, w](const int& state) {
                // Only update if the validity changes
                bool valid = w->validator(state);
                if (valid != w->current) {
                    w->current = valid;
                    emit(std::make_unique<StateUpdate>(id, w->type, state));
                }
            });
            w->handle.disable();

            // Add it to the list of when conditions
            provider->when.push_back(w);
        }
        else {
            throw std::runtime_error("When statements must come after a Provide statement");
        }
    }

    void Director::add_causing(const CausingExpression& causing) {
        auto it = providers.find(causing.reaction->id);
        if (it != providers.end()) {
            auto provider = it->second;

            // Can't add causing to a Start or Stop
            if (provider->classification == ProviderClassification::STOP
                || provider->classification == ProviderClassification::START) {
                throw std::runtime_error("You cannot use the 'Causing' DSL word with Start or Stop.");
            }

            provider->causing.emplace(causing.type, causing.resulting_state);
        }
        else {
            throw std::runtime_error("Causing statements must come after a Provide statement");
        }
    }

    void Director::add_needs(const NeedsExpression& needs) {
        auto it = providers.find(needs.reaction->id);

        if (it != providers.end()) {
            auto provider = it->second;

            // Can't add needs to a Start or Stop
            if (provider->classification == ProviderClassification::STOP
                || provider->classification == ProviderClassification::START) {
                throw std::runtime_error("You cannot use the 'Needs' DSL word with Start or Stop.");
            }

            provider->needs.emplace(needs.type);
        }
        else {
            throw std::runtime_error("Needs statements must come after a Provide statement");
        }
    }

    Director::Director(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        if (source != nullptr) {
            throw std::runtime_error("Multiple behaviour directors are not allowed.");
        }
        source = this;

        on<Configuration>("Director.yaml").then("Configure", [this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        // Removes all the Providers for a reaction when it is unbound
        on<Trigger<Unbind>, Sync<Director>>().then("Remove Provider", [this](const Unbind& unbind) {  //
            remove_provider(unbind.id);
        });

        // Add a Provider
        on<Trigger<ProvideReaction>, Sync<Director>>().then("Add Provider", [this](const ProvideReaction& provide) {
            add_provider(provide);
        });

        // Add a when expression to this Provider
        on<Trigger<WhenExpression>, Sync<Director>>().then("Add When", [this](const WhenExpression& when) {  //
            add_when(when);
        });

        // Add a causing condition to this Provider
        on<Trigger<CausingExpression>, Sync<Director>>().then("Add Causing", [this](const CausingExpression& causing) {
            add_causing(causing);
        });

        // Add a needs relationship to this Provider
        on<Trigger<NeedsExpression>, Sync<Director>>().then("Add Needs", [this](const NeedsExpression& needs) {  //
            add_needs(needs);
        });

        // A task has arrived, either it's a root task so we send it off immediately, or we build up our pack for when
        // the Provider has finished executing
        on<Trigger<BehaviourTask>, Sync<BehaviourTask>>().then(
            "Director Task",
            [this](const std::shared_ptr<const BehaviourTask>& task) {
                // Root level task, make the pack immediately and send it off to be executed as a root task
                if (providers.count(task->requester_id) == 0) {
                    emit(std::make_unique<TaskPack>(TaskPack({task})));
                }
                // Check if this Provider is active and allowed to make subtasks
                else if (providers[task->requester_id]->active) {
                    pack_builder.emplace(task->requester_task_id, task);
                }
                else {
                    // Throw an error so the user can see what a fool they are being
                    log<NUClear::WARN>("The task",
                                       task->name,
                                       "cannot be executed as the Provider",
                                       providers[task->requester_id]->reaction->identifier[0],
                                       "is not active and cannot make subtasks");
                }
            });

        // This reaction runs when a Provider finishes to send off the task pack to the main director
        on<Trigger<ProviderDone>, Sync<BehaviourTask>>().then("Package Tasks", [this](const ProviderDone& done) {
            // Get all the tasks that were emitted by this provider and send it as a task pack
            auto range = pack_builder.equal_range(done.requester_task_id);
            auto tasks = std::make_unique<TaskPack>();
            for (auto it = range.first; it != range.second; ++it) {
                tasks->push_back(it->second);
            }

            // Sort the task pack so highest priority tasks come first
            std::sort(tasks->begin(), tasks->end(), [](const auto& a, const auto& b) {
                return a->priority > b->priority;
            });

            // Emit the task pack
            emit(tasks);

            // Erase the task pack builder for this id
            pack_builder.erase(done.requester_task_id);
        });

        // A state that we were monitoring is updated, we might be able to run the task now
        on<Trigger<StateUpdate>, Sync<Director>>().then("State Updated", [this](const StateUpdate& update) {
            // Get the group that had a state update
            auto p  = providers[update.provider_id];
            auto& g = groups[p->type];

            // Go check if this state update has changed any of the tasks that are queued
            reevaluate_queue(g);
        });

        // We have a new task pack to run
        on<Trigger<TaskPack>, Sync<Director>>().then("Run Task Pack", [this](const TaskPack& pack) {  //
            run_task_pack(pack);
        });
    }

    Director::~Director() {
        // Remove this director as the information source
        source = nullptr;
    }

}  // namespace module::extension
