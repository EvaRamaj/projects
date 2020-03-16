"use strict";

import React from 'react';
import { Jumbotron } from 'reactstrap';
import { DataTable, TableHeader, TableBody, TableRow, TableColumn, Button } from 'react-md';
import { Card, FontIcon, TextField } from 'react-md';
import AuthService from '../services/AuthService';
import UserService from '../services/UserService';
import UserEvaluationService from '../services/UserEvaluationService';
import ConversationService from '../services/ConversationService';
import { MessageListRow } from './MessageListRow';
import StarRatingComponent from "react-star-rating-component"

const border = {
    borderRadius: "4",
    borderWidth: "0.5",
    borderColor: '#d6d7da',
    marginTop: 30
}
const pStyle = {
    marginLeft: "29%"
};

import { withRouter } from 'react-router-dom'

class MessageList extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            loading: true,
            body: '',
            recipientId: '',
            other_user: undefined,
            rating: 1
        };
        this.otherUser = {};
        this.handleChangeBody = this.handleChangeBody.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
    }

    componentWillMount(props){
       this.user = AuthService.getCurrentUser();
       var username = this.user.username;
       var others_id = undefined;
       this.setState(Object.assign({}, this.state, {username: username}));
        console.log("data",this.props.data[0].author.username)
        console.log("data",this.props.data[0].recipient.username)
       if (this.props.data[0].author.username !== username){
            others_id = this.props.data[0].author._id;
       }
       else{
           others_id = this.props.data[0].recipient._id;
       }
        console.log("other",others_id);
        console.log("me",this.user);
       UserService.getUser(others_id).then((data) => {
                this.otherUser = data;
                this.setState(Object.assign({}, this.state, {loading: false}));
           })
           .catch((e) => {
                console.error(e);
            });
    }
    handleChangeBody(event) {
        console.log(event.target)
        this.setState(Object.assign({}, this.state, {body: event.target.value}));
    }

    handleSubmit(event) {
        event.preventDefault();
        var conversation = {};
        conversation.body = this.state.body;
        conversation.conversationId = this.props.data[0].conversationId;
        ConversationService.reply(conversation).then((data) => {
                window.location.reload();
            }
        )
    }

    onStarClick(nextValue, prevValue, name) {
        this.setState({rating: nextValue});
    }

    render() {
        if(this.state.loading){
            return(<h2>Loading...</h2>)
        }
        const { rating } = this.state.rating;
        return (
            <div className="container" style={border}>
                <div className="row">
                    <div className="col-4">
                        <Jumbotron style={{width: 236}}>
                            <div className="fakeimg" style={{marginLeft: "2em"}}>
                                <a>
                                <img src={`http://localhost:3000/photos/${this.otherUser.photo}`} alt="Avatar" className="avatar1"/>
                                </a>
                            </div>
                            <h2 style={pStyle}>{this.otherUser.username}</h2>
                            <hr className="d-sm-none" />
                        </Jumbotron>
                    </div>
                    <div className="col-8">
                        {this.props.data.map((message, i) => <MessageListRow key={i} message={message} username={this.state.username}
                                                                             user={this.user} other_user={this.otherUser} id={this.props.id}/>)}
                        <div className="row">
                            <div className="col-9">
                                <input
                                    id="TitleField"
                                    type="text"
                                    className="md-row"
                                    required={true}
                                    value={this.state.body}
                                    onChange={this.handleChangeBody}/>
                            </div>
                            <div className="col-3">
                                <button id="submit" type="submit" onClick={this.handleSubmit}
                                        raised primary className="md-cell md-cell--2">Reply</button>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        );
    }
}

export default withRouter(MessageList);