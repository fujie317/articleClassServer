/**
 * service.cpp
 *
 *  Created on: Mar 27, 2017
 *      Author: Fujie Zhang
 */

#include <thread>
#include <atomic>
#include <iostream>
#include <memory>
#include <algorithm>
#include <fstream>
#include <vector>
#include <string>
#include <locale>
#include <codecvt>
#include <utility>
#include <future>
#include <bits/stdc++.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split
// #include "death_handler.h"

#define TITLE 4                                         // TITLE GRADE
#define KEYWORD 4                                       // KEYWORD GRADE
#define ABS 2                                           // ABSTRACT GRADE
#define MEET 1                                          // MEETING NAME GRADE
#define NumOfResults 12                                 // NO. OF RESULTS

const unsigned int DEFAULT_THREAD_POOL_SIZE = 4;

using namespace boost;

class Keyword {
public:
	unsigned int level=0;
	std::string word;
	double weight = 1.0;
	Keyword(){}
};


class Service {
public:
	Service(std::shared_ptr<asio::ip::tcp::socket> sock) :m_sock(sock){

	}

	void StartHandling(std::unordered_map<std::string, std::vector<Keyword>> &codes, std::vector<std::string> &clses) {

		asio::async_read_until(*m_sock.get(), m_request, '\n', [&, this](const boost::system::error_code& ec, std::size_t bytes_transferred){
			onRequestReceived(ec, bytes_transferred, codes, clses);
		});
	}

private:
	void onRequestReceived(const boost::system::error_code& ec,
			std::size_t bytes_transferred,
			std::unordered_map<std::string, std::vector<Keyword>> &codes,
			std::vector<std::string> &clses) {
		if (ec != 0) {
			std::cout << "Error occurred! Error code = "
					<< ec.value()
					<< ". Message: " << ec.message();

			onFinish();
			return;
		}

		// Process the request.
		m_response = ProcessRequest(m_request, codes, clses);

		// Initiate asynchronous write operation.
		asio::async_write(*m_sock.get(),
				asio::buffer(m_response),
				[&, this](
						const boost::system::error_code& ec,
						std::size_t bytes_transferred)
						{
			onResponseSent(ec,
					bytes_transferred);
						});
	}

	void onResponseSent(const boost::system::error_code& ec,	std::size_t bytes_transferred) {
		if (ec != 0) {
			std::cout << "Error occurred! Error code = " << ec.value() << ". Message: " << ec.message();
		}

		onFinish();
	}

	// Here we perform the cleanup.
	void onFinish() {
		delete this;
	}

	int KMP(std::string wtext, std::string wpattern){
		std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
		//  std::string narrow = converter.to_bytes(wide_utf16_source_string);
		std::wstring text = converter.from_bytes(wtext);
		std::wstring pattern = converter.from_bytes(wpattern);
		std::vector<int> pi(pattern.length() + 1, -1);
		int matches = 0;

		if(pattern.length() == 0){
			return 0;
		}
		int pos = -1;
		for(unsigned int i = 1; i <= pattern.length(); i++){
			while(pos >= 0 && pattern[pos] != pattern[i-1])
				pos = pi[pos];
			pi[i] = ++pos;
		}

		int kp = 0;
		for(int i = 0; i < text.length(); i++){
			while(kp >= 0 && pattern[kp] != text[i]){
				kp = pi[kp];
			}
			kp++;
			if(kp == pattern.length()){
				matches++;
				kp = pi[kp];
			}
		}
		return matches;
	}

	double clsScore(std::vector<std::string> &input, std::string cls, std::unordered_map<std::string, std::vector<Keyword>> &codes){
		if(input.size() != 4)
			return 0.0;
		double score = 0.0;
		double levelScore = 0.0;
		unsigned int level = 0;
		Keyword kw;
		double points;
		for(unsigned int i = 0; i < codes[cls].size(); ++i){         // work on each word
			kw = codes[cls][i];
			if(kw.level != level){
				if(!levelScore)
					break;
				level = kw.level;
				levelScore = 0.0;
			}
			points = 0.0;
			points += input[0].length() ? kw.weight / pow(2, kw.level) / input[0].length() * kw.word.length() * TITLE * KMP(input[0], kw.word) : 0;

			points += input[1].length() ? kw.weight / pow(2, kw.level) / input[1].length() * kw.word.length() * KEYWORD * KMP(input[1], kw.word) : 0;

			points += input[2].length() ? kw.weight / pow(2, kw.level) / input[2].length() * kw.word.length() * ABS * KMP(input[2], kw.word) : 0;

			if(!level)
				points += input[3].length() ? kw.weight / pow(2, kw.level) / input[3].length() * kw.word.length() * MEET * KMP(input[3], kw.word) : 0;

			score += points;
			levelScore += points;
		}
		return score;
	}

	std::vector<std::pair<double, std::string>> scores (std::vector<std::string> &input,
			std::unordered_map<std::string, std::vector<Keyword>> &codes,
			std::vector<std::string> &clses){
		std::vector<std::pair<double, std::string>>  results;
		unsigned int size = 0;
		unsigned int total = clses.size();
		double score;
		std::pair<double, std::string> p;
		for(size_t i = 0; i < total; ++i){
			score = clsScore(input, clses[i], codes);
			p = make_pair(score, clses[i]);
			if(size < NumOfResults){
				results.push_back(p);
				size++;
				if(size == NumOfResults)
					sort(results.begin(), results.end());
			}
			else if(results[0].first < p.first || (results[0].first == p.first && results[0].second < p.second)){
				results[0] = p;
				sort(results.begin(), results.end());
			}
		}
		return results;
	}

	std::vector <std::string> splitInput (std::string input){
		std::vector <std::string> output;
		size_t sindex, eindex;
		sindex = 0;
		while((eindex = input.find("##", sindex)) != std::string::npos){
			output.push_back(input.substr(sindex, eindex - sindex));
			sindex = eindex + 2;
		}
		output.push_back(input.substr(sindex));
		return output;
	}

	std::string ProcessRequest(asio::streambuf& request,
			std::unordered_map<std::string, std::vector<Keyword>> &codes,
			std::vector<std::string> &clses)
	{
		std::string input = to_string(request);
		input.erase(std::remove(input.begin(), input.end(), '\n'), input.end());
		std::vector<std::string> articleAttrs = splitInput(input);
		// Prepare and return the response message.
		std::vector<std::pair<double, std::string>> vecRslt = scores(articleAttrs, codes, clses);
		std::string response;
		for(int i = NumOfResults - 1; i > 0; --i)
			response += vecRslt[i].second + "##";
		response += vecRslt[0].second;

		return response;
	}

	std::string to_string(asio::streambuf &buf) {
		std::ostringstream out;
		out << &buf;
		return out.str();
	}

private:
	std::shared_ptr<asio::ip::tcp::socket> m_sock;
	std::string m_response;
	asio::streambuf m_request;
};


class Acceptor {
public:
	Acceptor(asio::io_service& ios, unsigned short port_num) :
		m_ios(ios),
		m_acceptor(m_ios,
				asio::ip::tcp::endpoint(
						asio::ip::address_v4::any(),
						port_num)),
						m_isStopped(false)
{}

	// Start accepting incoming connection requests.
	void Start(std::unordered_map<std::string, std::vector<Keyword>> &codes, std::vector<std::string> &clses) {
		m_acceptor.listen();
		InitAccept(codes, clses);
	}

	// Stop accepting incoming connection requests.
	void Stop() {
		m_isStopped.store(true);
	}

private:
	void InitAccept(std::unordered_map<std::string, std::vector<Keyword>> &codes, std::vector<std::string> &clses) {
		std::shared_ptr<asio::ip::tcp::socket> sock(new asio::ip::tcp::socket(m_ios));

		m_acceptor.async_accept(*sock.get(),[&, this, sock](const boost::system::error_code& error){onAccept(error, sock, codes, clses);});
	}

	void onAccept(const boost::system::error_code& ec,
			std::shared_ptr<asio::ip::tcp::socket> sock,
			std::unordered_map<std::string, std::vector<Keyword>> &codes,
			std::vector<std::string> &clses)
	{
		if (ec == 0) {
			(new Service(sock))->StartHandling(codes, clses);
		}
		else {
			std::cout << "Error occurred! Error code = "
					<< ec.value()
					<< ". Message: " << ec.message();
		}

		// Init next async accept operation if
		// acceptor has not been stopped yet.
		if (!m_isStopped.load()) {
			InitAccept(codes, clses);
		}
		else {
			// Stop accepting incoming connections
			// and free allocated resources.
			m_acceptor.close();
		}
	}

private:
	asio::io_service& m_ios;
	asio::ip::tcp::acceptor m_acceptor;
	std::atomic<bool> m_isStopped;
};


class Server {
public:
	Server() {
		m_work.reset(new asio::io_service::work(m_ios));
	}

	// Start the server.
	void Start(unsigned short port_num, unsigned int thread_pool_size, std::unordered_map<std::string, std::vector<Keyword>> &codes, std::vector<std::string> &clses) {
		assert(thread_pool_size > 0);

		// Create and start Acceptor.
		acc.reset(new Acceptor(m_ios, port_num));
		acc->Start(codes, clses);

		// Create specified number of threads and
		// add them to the pool.
		for (unsigned int i = 0; i < thread_pool_size; i++) {
			std::unique_ptr<std::thread> th(
					new std::thread([&, this]()
							{
				m_ios.run();
							}));

			m_thread_pool.push_back(std::move(th));
		}
		std::cout << "Server started" << std::endl;
	}

	// Stop the server.
	void Stop() {
		acc->Stop();
		m_ios.stop();

		for (auto& th : m_thread_pool) {
			th->join();
		}
		std::cout << "Server stopped" << std::endl;
	}

private:
	asio::io_service m_ios;
	std::unique_ptr<asio::io_service::work> m_work;
	std::unique_ptr<Acceptor> acc;
	std::vector<std::unique_ptr<std::thread>> m_thread_pool;
};


int main()
{
	//  Debug::DeathHandler dh;
	unsigned short port_num = 3333;
	std::ifstream file("Combo.txt");
	std::string line;
	std::unordered_map<std::string, std::vector<Keyword>> codes;
	std::vector<std::string> clses;
	while(std::getline(file, line)) {
		std::vector<std::string> combos;
		try{
			boost::split(combos, line, boost::is_any_of(" "), boost::token_compress_on);
		}
		catch(std::logic_error &le){
			std::cout << "Split combos, Logic error occurred"
					<< ". Message: "
					<< le.what();
		}
		auto size = combos.size();
		std::string cls = combos[0];
		clses.push_back(cls);
		std::vector<Keyword> tmpWords {};
		for(unsigned int i = 1; i < size; ++i){
			std::vector<std::string> kws;
			try{
				boost::split(kws, combos[i], boost::is_any_of("#"), boost::token_compress_on);
			}
			catch(std::logic_error &le){
				std::cout << "Split combos[i], Logic error occurred"
						<< ". Message: "
						<< le.what();
			}
			Keyword kw {};
			try{
				if(kws.size() > 1)
					kw.weight = stod(kws[1]);
				if(isdigit(kws[0][0])){
					kw.level = kws[0][0] - '0';
					kw.word = kws[0].substr(1);
				}
				else{
					kw.level = 0;
					kw.word = kws[0];
				}
			}
			catch(std::logic_error &le){
				std::cout << "keywords, Logic error occurred"
						<< ". Message: "
						<< le.what();
			}
			tmpWords.push_back(kw);
		}
		codes[cls] = tmpWords;
	}
	try {
		Server srv;

		unsigned int thread_pool_size =
				std::thread::hardware_concurrency() * 2;

		if (thread_pool_size == 0)
			thread_pool_size = DEFAULT_THREAD_POOL_SIZE;

		srv.Start(port_num, thread_pool_size, codes, clses);

		std::this_thread::sleep_for(std::chrono::seconds(5400000));

		srv.Stop();
	}
	catch (system::system_error &e) {
		std::cout << "Error occurred! Error code = "
				<< e.code() << ". Message: "
				<< e.what();
	}
	catch (std::logic_error &le){
		std::cout << "Logic error occurred"
				<< ". Message: "
				<< le.what();

	}

	return 0;
}





