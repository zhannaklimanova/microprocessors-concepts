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


#endif /* INC_TEST_CUSTOM_FUNCTIONS_H_ */
