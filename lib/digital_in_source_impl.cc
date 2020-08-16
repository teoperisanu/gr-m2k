/*
 * Copyright 2020 Analog Devices Inc.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "digital_in_source_impl.h"
#include "analog_in_source_impl.h"
#include <gnuradio/io_signature.h>
#include <libm2k/m2k.hpp>
#include <boost/lexical_cast.hpp>
#include <libm2k/m2kexceptions.hpp>


namespace gr {
namespace m2k {

digital_in_source::sptr
digital_in_source::make(const std::string &uri,
			int buffer_size,
			const int channel,
			double sampling_frequency,
			int kernel_buffers,
			bool streaming)
{
	return gnuradio::get_initial_sptr
		(new digital_in_source_impl(analog_in_source_impl::get_context(uri), buffer_size, channel, sampling_frequency, kernel_buffers, streaming));
}

digital_in_source::sptr
digital_in_source::make_from(libm2k::context::M2k *context,
                        int buffer_size,
                        const int channel,
                        double sampling_frequency,
                        int kernel_buffers,
                        bool streaming)
{
    return gnuradio::get_initial_sptr
            (new digital_in_source_impl(context, buffer_size, channel, sampling_frequency, kernel_buffers, streaming));
}

digital_in_source_impl::digital_in_source_impl(libm2k::context::M2k *context,
					       int buffer_size,
					       const int channel,
					       double sampling_frequency,
					       int kernel_buffers,
					       bool streaming)
	: gr::sync_block("digital_in_source",
			 gr::io_signature::make(0, 0, 0),
			 gr::io_signature::make(1, 1, sizeof(uint16_t))),
	d_uri(context->getUri()),
	d_buffer_size(buffer_size),
	d_channel(channel)
{
    analog_in_source_impl::add_context(context);
	d_digital = context->getDigital();

	d_digital->setKernelBuffersCountIn(kernel_buffers);
	set_params(sampling_frequency, streaming);

	d_items_in_buffer = 0;
	set_output_multiple(0x400);
}

digital_in_source_impl::~digital_in_source_impl()
{
//	analog_in_source_impl::remove_contexts(d_uri);
}

void digital_in_source_impl::set_params(double sampling_frequency, bool streaming)
{
	d_digital->setSampleRateIn(sampling_frequency);
	auto trigger = d_digital->getTrigger();
	trigger->setDigitalStreamingFlag(streaming);
}

int digital_in_source_impl::work(int noutput_items,
				gr_vector_const_void_star &input_items,
				gr_vector_void_star &output_items)
{
	boost::unique_lock<boost::mutex> lock(d_buffer_mutex);

	if (!d_items_in_buffer) {
		try {
			d_raw_samples = d_digital->getSamplesP(d_buffer_size);

			std::cout << "digital captured data!" << std::endl;

		} catch (m2k_exception &e) {
//			message_port_pub(d_port_id, pmt::mp("timeout"));
			// tmp: ==============================================
			pmt::pmt_t payload = pmt::from_long(0);
			pmt::pmt_t msg = pmt::cons(pmt::mp("done"), payload);
			post(pmt::mp("system"), msg);
			// ===================================================
			return 0;
		} catch (std::exception &e) {
			std::cout << e.what() << std::endl;
			return 0;
		}
		d_items_in_buffer = (unsigned long) d_buffer_size;
		d_sample_index = 0;
	}

	unsigned long nb_samples = std::min(d_items_in_buffer, (unsigned long) noutput_items);
	unsigned int out_stream_index = 0;

    if (!d_sample_index) {
        tag_t tag;
        tag.value = pmt::from_long(d_items_in_buffer);
        tag.offset = nitems_written(out_stream_index);
        tag.key = pmt::intern("buffer_start");
        tag.srcid = alias_pmt();

        add_item_tag(out_stream_index, tag);
    }
    uint16_t *out = (uint16_t *) output_items[out_stream_index];
    memcpy(out, d_raw_samples + d_sample_index, sizeof(uint16_t) * nb_samples);
	d_items_in_buffer -= nb_samples;
	d_sample_index += nb_samples;

	return (int) nb_samples;
}

unsigned short digital_in_source_impl::get_channel_value(unsigned int channel, unsigned short sample)
{
	return (sample & (1u << channel )) >> channel;
}

void gr::m2k::digital_in_source_impl::set_buffer_size(int buffer_size)
{
	if (d_buffer_size != buffer_size) {
		boost::unique_lock<boost::mutex> lock(d_buffer_mutex);

		d_items_in_buffer = 0;
		d_buffer_size = buffer_size;
	}
}

} /* namespace m2k */
} /* namespace gr */
