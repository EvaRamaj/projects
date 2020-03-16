"use strict";

import React from 'react';

import { ConversationDetail } from '../components/ConversationDetail';

import ConversationService from '../services/ConversationService';
import UserService from '../services/UserService';
import AuthService from '../services/AuthService';


export class ConversationDetailView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            loading: true,
            username: '',
            _id:'',
            conversation: []
        };
    }

    componentWillMount(props){
        let id = this.props.match.params.id;
        this.users = [];
        ConversationService.getConversation(id).then((data) => {
                this.setState({
                    loading: true,
                    username: '',
                    _id:'',
                    conversation: data.conversations,
                });
                var my_id = AuthService.getCurrentUser().id;
                if(data.conversations[0].participants[0] === my_id.toString()){
                    console.log("in8")
                    UserService.getUser(data.conversations[0].participants[1]).then((data) => {
                        this.setState(Object.assign({}, this.state, {username: data.username}));
                        this.setState(Object.assign({}, this.state, {_id: data._id}));
                    }).catch((e) => {
                        console.error(e);
                    });
                }
                else{
                    UserService.getUser(data.conversations[0].participants[0]).then((data) => {
                        console.log("in10",data)
                        this.setState(Object.assign({}, this.state, {username: data.username}));
                        this.setState(Object.assign({}, this.state, {_id: data._id}));
                    }).catch((e) => {
                        console.error(e);
                    });
                }
            this.setState(Object.assign({}, this.state, {loading: false}));
            }).catch((e) => {
                console.error(e);
            });
        }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }
        console.log("state",this.state)
        return (
            <ConversationDetail conversation={this.state.conversation} username={this.state.username} id={this.state._id}/>
        );
    }
}
