#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <sstream>
#include <boost/core/demangle.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/profile_dictionary.h>

namespace tesseract_planning::tmp
{
template <typename ProfileT>
ProfileDictionary<ProfileT>::ProfileDictionary(const ProfileDictionary& rhs) : profiles_(rhs.profiles_)
{
}

template <typename ProfileT>
ProfileDictionary<ProfileT>::ProfileDictionary(ProfileDictionary&& rhs) noexcept : profiles_(std::move(rhs.profiles_))
{
}

template <typename ProfileT>
ProfileDictionary<ProfileT>& ProfileDictionary<ProfileT>::operator=(const ProfileDictionary& rhs)
{
  profiles_ = rhs.profiles_;
  return *this;
}

template <typename ProfileT>
ProfileDictionary<ProfileT>& ProfileDictionary<ProfileT>::operator=(ProfileDictionary&& rhs) noexcept
{
  profiles_ = std::move(rhs.profiles_);
  return *this;
}

template <typename ProfileT>
bool ProfileDictionary<ProfileT>::hasProfile(const std::string& ns, const std::string& profile) const
{
  std::shared_lock lock(mutex_);

  auto ns_it = profiles_.find(ns);
  if (ns_it == profiles_.end())
    return false;

  auto prof_it = ns_it->second.find(profile);
  return prof_it != ns_it->second.end();
}

template <typename ProfileT>
typename ProfileT::ConstPtr ProfileDictionary<ProfileT>::getProfile(const std::string& ns,
                                                                    const std::string& profile) const
{
  std::shared_lock lock(mutex_);
  auto ns_it = profiles_.find(ns);
  if (ns_it == profiles_.end())
    throw std::runtime_error("Profile namespace does not exist for '" + ns + "'!");

  auto prof_it = ns_it->second.find(profile);
  if (prof_it == ns_it->second.end())
    throw std::runtime_error("Profile entry does not exist for profile '" + profile + "' in namespace '" + ns + "'");

  return prof_it->second;
}

template <typename ProfileT>
std::unordered_map<std::string, typename ProfileT::ConstPtr>
ProfileDictionary<ProfileT>::getProfileEntry(const std::string& ns) const
{
  std::shared_lock lock(mutex_);
  auto ns_it = profiles_.find(ns);
  if (ns_it == profiles_.end())
    throw std::runtime_error("Profile namespace does not exist for '" + ns + "'!");

  return ns_it->second;
}

template <typename ProfileT>
void ProfileDictionary<ProfileT>::addProfile(const std::string& ns,
                                             const std::string& profile_name,
                                             typename ProfileT::ConstPtr profile)
{
  if (ns.empty())
    throw std::runtime_error("Namespace cannot be empty");

  if (profile_name.empty())
    throw std::runtime_error("Profile name cannot be empty");

  if (profile == nullptr)
    throw std::runtime_error("Profile cannot be a nullptr");

  std::unique_lock lock(mutex_);
  auto it = profiles_.find(ns);
  if (it == profiles_.end())
  {
    // Create a new map for the namespace
    profiles_[ns] = { { profile_name, profile } };
  }
  else
  {
    profiles_[ns][profile_name] = profile;
  }
}

template <typename ProfileT>
void ProfileDictionary<ProfileT>::removeProfile(const std::string& ns, const std::string& profile)
{
  std::unique_lock lock(mutex_);

  auto it = profiles_.find(ns);
  if (it != profiles_.end())
    it->second.erase(profile);
}

template class ProfileDictionary<WaypointProfile>;
template class ProfileDictionary<CompositeProfile>;
template class ProfileDictionary<PlannerProfile>;

}  // namespace tesseract_planning::tmp
