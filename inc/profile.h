#ifndef __PROFILE_H
#define __PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/*
 * Configure
 * - C10
 * as the profile GPIO so we can time stuff more precisely.
 */
void profile_init();
void profile_toggle();


#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __PROFILE_H

