/*
 * Copyright © 2013 Keith Packard <keithp@keithp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef _AO_QUATERNION_H_
#define _AO_QUATERNION_H_

#include <math.h>

struct ao_quaternion {
	float	r;		/* real bit */
	float	x, y, z;	/* imaginary bits */
};

static inline void ao_quaternion_multiply(struct ao_quaternion *r,
					  const struct ao_quaternion *a,
					  const struct ao_quaternion *b)
{
	struct ao_quaternion	t;
#define T(_a,_b)	(((a)->_a) * ((b)->_b))

/*
 * Quaternions
 *
 *	ii = jj = kk = ijk = -1;
 *
 *	kji = 1;
 *
 * 	ij = k;		ji = -k;
 *	kj = -i;	jk = i;
 *	ik = -j;	ki = j;
 *
 * Multiplication p * q:
 *
 *	(pr + ipx + jpy + kpz) (qr + iqx + jqy + kqz) =
 *
 *		( pr * qr +  pr * iqx +  pr * jqy +  pr * kqz) +
 *		(ipx * qr + ipx * iqx + ipx * jqy + ipx * kqz) +
 *		(jpy * qr + jpy * iqx + jpy * jqy + jpy * kqz) +
 *		(kpz * qr + kpz * iqx + kpz * jqy + kpz * kqz) =
 *
 *
 *		 (pr * qr) + i(pr * qx) + j(pr * qy) + k(pr * qz) +
 *		i(px * qr) -  (px * qx) + k(px * qy) - j(px * qz) +
 *		j(py * qr) - k(py * qx) -  (py * qy) + i(py * qz) +
 *		k(pz * qr) + j(pz * qx) - i(pz * qy) -  (pz * qz) =
 *
 *		1 * ( (pr * qr) - (px * qx) - (py * qy) - (pz * qz) ) +
 *		i * ( (pr * qx) + (px * qr) + (py * qz) - (pz * qy) ) +
 *		j * ( (pr * qy) - (px * qz) + (py * qr) + (pz * qx) ) +
 *		k * ( (pr * qz) + (px * qy) - (py * qx) + (pz * qr);
 */

	t.r = T(r,r) - T(x,x) - T(y,y) - T(z,z);
	t.x = T(r,x) + T(x,r) + T(y,z) - T(z,y);
	t.y = T(r,y) - T(x,z) + T(y,r) + T(z,x);
	t.z = T(r,z) + T(x,y) - T(y,x) + T(z,r);
#undef T
	*r = t;
}

static inline void ao_quaternion_conjugate(struct ao_quaternion *r,
					   const struct ao_quaternion *a)
{
	r->r = a->r;
	r->x = -a->x;
	r->y = -a->y;
	r->z = -a->z;
}

static inline float ao_quaternion_normal(const struct ao_quaternion *a)
{
#define S(_a)	(((a)->_a) * ((a)->_a))
	return S(r) + S(x) + S(y) + S(z);
#undef S
}

static inline void ao_quaternion_scale(struct ao_quaternion *r,
				       const struct ao_quaternion *a,
				       float b)
{
	r->r = a->r * b;
	r->x = a->x * b;
	r->y = a->y * b;
	r->z = a->z * b;
}

static inline void ao_quaternion_normalize(struct ao_quaternion *r,
					   const struct ao_quaternion *a)
{
	float	n = ao_quaternion_normal(a);

	if (n > 0)
		ao_quaternion_scale(r, a, 1/sqrtf(n));
	else
		*r = *a;
}

static inline float ao_quaternion_dot(const struct ao_quaternion *a,
				      const struct ao_quaternion *b)
{
#define T(_a)	(((a)->_a) * ((b)->_a))
	return T(r) + T(x) + T(y) + T(z);
#undef T
}
				     

static inline void ao_quaternion_rotate(struct ao_quaternion *r,
					const struct ao_quaternion *a,
					const struct ao_quaternion *b)
{
	struct ao_quaternion	c;
	struct ao_quaternion	t;

	ao_quaternion_multiply(&t, b, a);
	ao_quaternion_conjugate(&c, b);
	ao_quaternion_multiply(r, &t, &c);
}

/*
 * Compute a rotation quaternion between two vectors
 *
 *	cos(θ) + u * sin(θ)
 *
 * where θ is the angle between the two vectors and u
 * is a unit vector axis of rotation
 */

static inline void ao_quaternion_vectors_to_rotation(struct ao_quaternion *r,
						     const struct ao_quaternion *a,
						     const struct ao_quaternion *b)
{
	/*
	 * The cross product will point orthogonally to the two
	 * vectors, forming our rotation axis. The length will be
	 * sin(θ), so these values are already multiplied by that.
	 */

	float x = a->y * b->z - a->z * b->y;
	float y = a->z * b->x - a->x * b->z;
	float z = a->x * b->y - a->y * b->x;

	float s_2 = x*x + y*y + z*z;
	float s = sqrtf(s_2);

	/* cos(θ) = a · b / (|a| |b|).
	 *
	 * a and b are both unit vectors, so the divisor is one
	 */
	float c = a->x*b->x + a->y*b->y + a->z*b->z;

	float c_half = sqrtf ((1 + c) / 2);
	float s_half = sqrtf ((1 - c) / 2);

	/*
	 * Divide out the sine factor from the
	 * cross product, then multiply in the
	 * half sine factor needed for the quaternion
	 */
	float s_scale = s_half / s;

	r->x = x * s_scale;
	r->y = y * s_scale;
	r->z = z * s_scale;

	r->r = c_half;

	ao_quaternion_normalize(r, r);
}

static inline void ao_quaternion_init_vector(struct ao_quaternion *r,
					     float x, float y, float z)
{
	r->r = 0;
	r->x = x;
	r->y = y;
	r->z = z;
}

static inline void ao_quaternion_init_rotation(struct ao_quaternion *r,
					       float x, float y, float z,
					       float s, float c)
{
	r->r = c;
	r->x = s * x;
	r->y = s * y;
	r->z = s * z;
}

static inline void ao_quaternion_init_zero_rotation(struct ao_quaternion *r)
{
	r->r = 1;
	r->x = r->y = r->z = 0;
}

/*
 * The sincosf from newlib just calls sinf and cosf. This is a bit
 * faster, if slightly less precise
 */

static inline void
ao_sincosf(float a, float *s, float *c) {
	float	_s = sinf(a);
	*s = _s;
	*c = sqrtf(1 - _s*_s);
}

/*
 * Initialize a quaternion from 1/2 euler rotation angles (in radians).
 *
 * Yes, it would be nicer if there were a faster way, but because we
 * sample the gyros at only 100Hz, we end up getting angles too large
 * to take advantage of sin(x) ≃ x.
 *
 * We might be able to use just a couple of elements of the sin taylor
 * series though, instead of the whole sin function?
 */

static inline void ao_quaternion_init_half_euler(struct ao_quaternion *r,
						 float x, float y, float z)
{
	float	s_x, c_x;
	float	s_y, c_y;
	float	s_z, c_z;

	ao_sincosf(x, &s_x, &c_x);
	ao_sincosf(y, &s_y, &c_y);
	ao_sincosf(z, &s_z, &c_z);

	r->r = c_x * c_y * c_z + s_x * s_y * s_z;
	r->x = s_x * c_y * c_z - c_x * s_y * s_z;
	r->y = c_x * s_y * c_z + s_x * c_y * s_z;
	r->z = c_x * c_y * s_z - s_x * s_y * c_z;
}

#endif /* _AO_QUATERNION_H_ */