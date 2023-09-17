/*
 * test_custom_functions.h
 *
 *  Created on: Sep 16, 2023
 *      Author: nafiz
 */

#ifndef INC_TEST_CUSTOM_FUNCTIONS_H_
#define INC_TEST_CUSTOM_FUNCTIONS_H_

#define TEST_SQRT_RANGE(port, findSqrt, start, end, step, port_value) \
		for (float32_t i = start; i < end; i += step) { \
			float32_t sqrt; \
			findSqrt(i * i, &sqrt); \
			double diff = fabs(sqrt * sqrt - i * i); \
			if (diff > tolerance) { \
				port = port_value; \
				break; \
			} \
		}
#define TEST_SQRT(port, findSqrt, portStart) \
		do { \
			double tolerance = 0.00005; \
			{ \
			float32_t i = -1.0; \
			float32_t sqrt; \
			findSqrt(i, &sqrt); \
			if (!isnan(sqrt)) { \
				port = portStart + 1; \
			} \
			TEST_SQRT_RANGE(port, findSqrt, 0.001, 1, 0.001, portStart + 2); \
			TEST_SQRT_RANGE(port, findSqrt, 0.1, 10, 0.1, portStart + 3); \
			TEST_SQRT_RANGE(port, findSqrt, 1, 100, 1, portStart + 4); \
			} \
		} while (0);

#define TEST_TRANSCENDENTAL(port, findTranscendental, id) \
		float32_t actual; \
		float32_t expected; \
		do { \
			double tolerance = 0.0001; \
			omega = -PI; \
			for (; omega < PI; omega += 0.001) { \
				phi = 0; \
					float32_t solution; \
					findTranscendental(omega, phi, &solution); \
					actual = solution * solution; \
					expected = arm_cos_f32(omega*(solution) + phi); \
					double diff = fabs(expected - actual); \
					if (diff > tolerance) { \
						port = id; \
						goto go_to_label_##id; \
					} \
			} \
		} while (0); \
		go_to_label_##id:

#endif /* INC_TEST_CUSTOM_FUNCTIONS_H_ */
