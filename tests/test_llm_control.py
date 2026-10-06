import io
import json
import sys
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from llm_control import build_messages, normalize_plan, request_plan


class LLMControl(unittest.TestCase):
    def test_prompt_contains_camera_and_original_user_request(self):
        camera = {'valid': True, 'zones': {
            'zone_1_red': {'state': 'occupied', 'object_colors': ['green']},
            'zone_Intermediate': {'state': 'empty', 'object_colors': []}}}
        request = 'Đặt vật đỏ vào vùng xanh lá.'
        template = 'Có 3 function skill: place(object), pick(object,zone), home.'
        self.assertEqual(build_messages(template, camera, request), [
            {'role': 'system', 'content': template + '\n\nCAMERA_CHECK\n'
             + json.dumps(camera, ensure_ascii=False)},
            {'role': 'user', 'content': request}])

    def test_chat_request_uses_9router_contract(self):
        messages = build_messages('Prompt', {'valid': True, 'zones': {}}, 'Về home')
        body = json.dumps({'choices': [{'message': {'content': 'home'}}]}).encode()
        with patch('llm_control.urlopen', return_value=io.BytesIO(body)) as send:
            self.assertEqual(request_plan('http://localhost:20128/v1/',
                                          'test-key', 'ur3e', messages, 15.), 'home')
        send.assert_called_once()
        http_request = send.call_args.args[0]
        self.assertEqual(http_request.full_url,
                         'http://localhost:20128/v1/chat/completions')
        self.assertEqual(http_request.get_method(), 'POST')
        self.assertEqual(http_request.get_header('Authorization'), 'Bearer test-key')
        self.assertEqual(json.loads(http_request.data), {
            'model': 'ur3e', 'messages': messages, 'stream': False})
        self.assertEqual(send.call_args.kwargs['timeout'], 15.)

    def test_empty_or_malformed_responses_are_rejected(self):
        responses = [b'not json', b'null', b'[]', b'{}', b'{"choices": []}']
        for content in (None, '', '   ', ['home'], {'plan': 'home'}):
            responses.append(json.dumps({
                'choices': [{'message': {'content': content}}]}).encode())
        for response in responses:
            with self.subTest(response=response):
                with patch('llm_control.urlopen', return_value=io.BytesIO(response)):
                    with self.assertRaises(ValueError):
                        request_plan('http://localhost:20128/v1',
                                     'test-key', 'ur3e', [], 15.)

    def test_http_error_is_generic_and_is_not_retried(self):
        provider_secret = 'provider-private-token'
        error = urllib.error.HTTPError(
            'http://localhost:20128/v1/chat/completions', 403,
            provider_secret, {}, io.BytesIO(provider_secret.encode()))
        with patch('llm_control.urlopen', side_effect=error) as send:
            with self.assertRaises(ValueError) as raised:
                request_plan('http://localhost:20128/v1',
                             'test-key', 'ur3e', [], 15.)
        send.assert_called_once()
        self.assertIn('403', str(raised.exception))
        self.assertNotIn(provider_secret, str(raised.exception))
        self.assertNotIn('test-key', str(raised.exception))

    def test_plan_accepts_existing_cli_aliases(self):
        self.assertEqual(normalize_plan(
            '  pick green zone_Intermediate;\n'
            'pick red_box zone_1_red; place purple; home  '),
            'pick green zone_Intermediate; pick red_box zone_1_red; place purple; home')
        for colour in ('red', 'yellow', 'green', 'blue', 'purple'):
            for object_name in (colour, colour + '_box'):
                self.assertEqual(normalize_plan('place ' + object_name),
                                 'place ' + object_name)
        for zone in ('gray', 'red', 'green', 'yellow', 'zone_Intermediate',
                     'zone_1_red', 'zone_2_green', 'zone_3_yellow'):
            self.assertEqual(normalize_plan('pick blue ' + zone), 'pick blue ' + zone)

    def test_plan_rejects_unknown_commands_and_parameters(self):
        for command in ('', 'move red', 'place pink', 'pick red zone_3_blue',
                        'pick red', 'home red', 'place --ros-args',
                        'place(red)', '```\nhome\n```', 'home; echo secret',
                        'place "red'):
            with self.subTest(command=command), self.assertRaises(ValueError):
                normalize_plan(command)


if __name__ == '__main__':
    unittest.main()
